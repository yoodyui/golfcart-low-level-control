#!/usr/bin/env python3
"""
ros_can_bridge — supervisor + ROS2 <-> CAN bridge (runs on Jetson)

Flow ต่อ 1 รอบ control loop (50 Hz):
  1) รับ /cmd_vel (Twist) จาก nav/teleop
  2) อ่าน e-stop จาก brake board heartbeat (CAN 0x131 byte[5]) -> /e_stop_status
  3) สร้าง condition (e-stop / cmd_vel timeout / board fault)
  4) จากนั้นค่อยสั่ง steering (/steering_cmd), brake (0x130), speed (0x120/0x121)

cmd_vel.angular.z = yaw rate (rad/s) จริง -> แปลงเป็นมุมเลี้ยวด้วย bicycle model
  delta = atan(wheelbase * yaw_rate / v)   (มี guard กันหารด้วยศูนย์)

Wire format (ตรงกับ MotorBrake_Firmware_Car_Shield/Core/Src/main.c):
  TX 0x120  int16 LE, 0.01 m/s/LSB   target speed
  TX 0x121  byte0: 1=enable PID      speed enable
  TX 0x130  byte0: 1=brake 0=release brake command
  RX 0x122  int16 LE, 0.01 m/s/LSB   measured speed
  RX 0x131  [0..3] float32 LE current_mA, [4] relay_active,
            [5] watchdog/e-stop (0=Normal, 1=Triggered), [6..7] uint16 seq

Safety rules (diagram V2):
  1) e-stop (จาก 0x131)   -> enable=0, speed=0, brake=1
  2) /cmd_vel timeout      -> enable=0, speed=0, brake=1
  3) board fault/heartbeat -> enable=0, speed=0, brake=1
  4) normal                -> brake=0, enable=1, speed=linear.x, /steering_cmd

Before running:  sudo ip link set can0 up type can bitrate 250000
"""

import math
import struct
import threading

import can
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, Float32, UInt8

CAN_ID_SPEED_CMD    = 0x120
CAN_ID_SPEED_ENABLE = 0x121
CAN_ID_SPEED_STATUS = 0x122
CAN_ID_BRAKE_CMD    = 0x130
CAN_ID_BRAKE_STATUS = 0x131

SPEED_SCALE = 100.0          # int16 raw = m/s * 100


class RosCanBridge(Node):
    def __init__(self):
        super().__init__('ros_can_bridge')

        # ---------- parameters ----------
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)
        self.declare_parameter('board_timeout_s', 0.5)     # 0x131 มาทุก 20 ms
        self.declare_parameter('wheelbase_m', 1.65)        # <- วัดจากรถจริง
        self.declare_parameter('max_speed_mps', 5.0)
        self.declare_parameter('max_steering_deg', 50.0)
        # ต่ำกว่าความเร็วนี้จะไม่คำนวณ atan (กันหารด้วยศูนย์ / มุมระเบิดที่ v ต่ำ)
        self.declare_parameter('min_speed_for_steering_mps', 0.1)

        ch          = self.get_parameter('can_channel').value
        rate        = self.get_parameter('control_rate_hz').value
        self.cmd_to = self.get_parameter('cmd_vel_timeout_s').value
        self.brd_to = self.get_parameter('board_timeout_s').value
        self.wheelbase = self.get_parameter('wheelbase_m').value
        self.max_speed = self.get_parameter('max_speed_mps').value
        self.max_steer = math.radians(self.get_parameter('max_steering_deg').value)
        self.min_v     = self.get_parameter('min_speed_for_steering_mps').value

        # ---------- CAN ----------
        self.bus = can.Bus(channel=ch, interface='socketcan')
        self._rx_stop = threading.Event()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)

        # ---------- state ----------
        self.cmd = Twist()
        self.last_cmd_stamp = None          # None = ยังไม่เคยได้ /cmd_vel
        self.estop_active = True            # เริ่มแบบ fail-safe จนกว่า 0x131 จะบอกว่าปกติ
        self.last_board_stamp = None
        self.last_steer = 0.0
        self.measured_speed = 0.0

        # ---------- ROS I/O ----------
        self.create_subscription(Twist, '/cmd_vel', self._cb_cmd_vel, 10)

        self.pub_steer_cmd   = self.create_publisher(Float64, '/steering_cmd', 10)
        self.pub_estop       = self.create_publisher(Bool,   '/e_stop_status', 10)
        self.pub_speed_meas  = self.create_publisher(Float64, '/speed_status', 10)
        self.pub_brake_state = self.create_publisher(UInt8,  '/brake_status', 10)
        self.pub_brake_cur   = self.create_publisher(Float32, '/brake_current', 10)
        self.pub_fault       = self.create_publisher(Bool,   '/lowlevel_fault', 10)

        self.create_timer(1.0 / rate, self._control_loop)
        self._rx_thread.start()
        self.get_logger().info(f'ros_can_bridge up on {ch} (angular.z = yaw rate)')

    # ================= ROS callbacks =================
    def _cb_cmd_vel(self, msg: Twist):
        self.cmd = msg
        self.last_cmd_stamp = self.get_clock().now()

    # ================= main control loop =================
    def _control_loop(self):
        now = self.get_clock().now()

        # ---- step 1-3: สร้าง condition ก่อน ----
        cmd_timeout = (
            self.last_cmd_stamp is None
            or (now - self.last_cmd_stamp).nanoseconds * 1e-9 > self.cmd_to
        )
        board_lost = (
            self.last_board_stamp is None
            or (now - self.last_board_stamp).nanoseconds * 1e-9 > self.brd_to
        )
        stop_required = self.estop_active or cmd_timeout or board_lost
        self.pub_fault.publish(Bool(data=bool(stop_required)))

        # ---- step 4: สั่ง actuator ตาม condition ----
        if stop_required:
            # rules 1-3: หยุด + เบรก, steering ค้างมุมเดิม (ไม่ publish)
            self._send_enable(0)
            self._send_speed(0.0)
            self._send_brake(1)
            return

        v = float(self.cmd.linear.x)
        v = max(-self.max_speed, min(self.max_speed, v))

        if abs(v) > 1e-3:
            self._send_brake(0)
            self._send_enable(1)
            self._send_speed(v)
        else:
            # ผู้ใช้สั่งหยุด (linear.x = 0) -> ปล่อยคันเร่ง + เบรก
            self._send_enable(0)
            self._send_speed(0.0)
            self._send_brake(1)

        steer = self._compute_steering(v)
        self.last_steer = steer
        self.pub_steer_cmd.publish(Float64(data=steer))

    def _compute_steering(self, v: float) -> float:
        """bicycle model: delta = atan(L * yaw_rate / v) พร้อม guard หารด้วยศูนย์"""
        yaw_rate = float(self.cmd.angular.z)

        if abs(v) < self.min_v:
            # ความเร็วต่ำ/ศูนย์: ห้ามหาร
            if abs(yaw_rate) < 1e-3:
                return 0.0              # ไม่ขยับ + ไม่สั่งเลี้ยว -> คืนพวงมาลัยตรง
            return self.last_steer      # อยากเลี้ยวแต่รถแทบไม่วิ่ง -> ค้างมุมเดิม

        steer = math.atan(self.wheelbase * yaw_rate / v)
        return max(-self.max_steer, min(self.max_steer, steer))

    # ================= CAN TX =================
    def _tx(self, can_id: int, data: bytes):
        try:
            self.bus.send(can.Message(arbitration_id=can_id, data=data,
                                      is_extended_id=False))
        except can.CanError as e:
            self.get_logger().error(f'CAN TX 0x{can_id:03X} failed: {e}',
                                    throttle_duration_sec=1.0)

    def _send_speed(self, mps: float):
        raw = int(round(mps * SPEED_SCALE))
        raw = max(-32768, min(32767, raw))
        self._tx(CAN_ID_SPEED_CMD, struct.pack('<h', raw))

    def _send_enable(self, en: int):
        self._tx(CAN_ID_SPEED_ENABLE, bytes([1 if en else 0]))

    def _send_brake(self, b: int):
        self._tx(CAN_ID_BRAKE_CMD, bytes([1 if b else 0]))

    # ================= CAN RX (thread) =================
    def _rx_loop(self):
        while not self._rx_stop.is_set():
            msg = self.bus.recv(timeout=0.1)
            if msg is None or msg.is_extended_id:
                continue                       # extended = KEYA, ปล่อยให้ steering_node

            if msg.arbitration_id == CAN_ID_SPEED_STATUS and len(msg.data) >= 2:
                (raw,) = struct.unpack_from('<h', bytes(msg.data), 0)
                self.measured_speed = raw / SPEED_SCALE
                self.pub_speed_meas.publish(Float64(data=self.measured_speed))

            elif msg.arbitration_id == CAN_ID_BRAKE_STATUS and len(msg.data) >= 8:
                d = bytes(msg.data)
                current_ma = struct.unpack_from('<f', d, 0)[0]
                relay      = d[4]
                estop      = bool(d[5])        # watchdog/e-stop byte จากบอร์ดเบรก
                self.last_board_stamp = self.get_clock().now()

                if estop and not self.estop_active:
                    self.get_logger().warn('E-STOP from brake board (0x131)')
                self.estop_active = estop

                self.pub_estop.publish(Bool(data=estop))
                self.pub_brake_state.publish(UInt8(data=relay))
                self.pub_brake_cur.publish(Float32(data=current_ma))

    def destroy_node(self):
        self._rx_stop.set()
        self._rx_thread.join(timeout=1.0)
        try:
            # fail-safe ก่อนปิด node
            self._send_enable(0)
            self._send_speed(0.0)
            self._send_brake(1)
            self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosCanBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
