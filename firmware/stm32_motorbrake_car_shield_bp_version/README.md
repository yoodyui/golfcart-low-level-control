# STM32 Brake + Speed Firmware — bp_version

เฟิร์มแวร์ STM32G474 (MotorBrake board + Logger/DataManager shield) รุ่นที่แก้เรื่องความปลอดภัยของเบรก
ตอนขับแมนนวล, สวิตช์โหมด MANUAL/AUTO และปุ่ม E-Stop

- ฐานโค้ด: `MotorBrake_Firmware_Car_Shield` เวอร์ชันวันที่ 26 ก.ย. 2026 (ใหม่กว่าโฟลเดอร์
  `../stm32_motorbrake_car_shield` ใน repo นี้ — มี auto-brake, `/brake_angle` 0x134 ฯลฯ)
- แก้เฉพาะ `Core/Src/main.c` และทุกจุดอยู่ในบล็อก `USER CODE` ค้นหา `[BP-` จะเจอ 28 บรรทัด
- รายละเอียดการแก้แต่ละข้อ: [`CHANGES_BP.md`](CHANGES_BP.md)
- คอมไพล์และลิงก์ผ่าน (arm-none-eabi-gcc 13.2) — **ยังไม่ได้ทดสอบบนรถจริง**

## ปัญหาที่แก้

เฟิร์มแวร์เดิม: เมื่อ ROS เคยส่ง `/cmd_vel = 0` แม้ครั้งเดียว ถ้าคนขับขับเองแล้วล้อหมุนเกิน 0.05 m/s
เซอร์โวจะดึงเบรกสู้คนขับ (auto-brake ไม่ได้เช็กโหมด / speed_enable / cmd_vel หมดอายุ)

## ฮาร์ดแวร์ที่ต้องเตรียมก่อนแฟลช

ดูรูป: [`docs/bp_version/motorbrake_board_annotated.png`](../../docs/bp_version/motorbrake_board_annotated.png),
[`docs/bp_version/logger_shield_annotated.png`](../../docs/bp_version/logger_shield_annotated.png),
[`docs/bp_version/estop_wiring.png`](../../docs/bp_version/estop_wiring.png) (วิธีต่อปุ่ม E-Stop)

| จุด | ทำอะไร | ถ้าไม่ทำ |
|---|---|---|
| **CN3 EMER** (บอร์ดล่าง) | เสียบจัมเปอร์ค้างไว้ | เซอร์โวไม่มีไฟ เบรกใช้ไม่ได้ |
| **PC13** = ขา 7 ของ AUX_PORT (= J2 ขา 7 บน Logger Shield; คอลัมน์ขาคี่ ขาที่ 4 นับจากขา 1 = +5V) | ต่อหน้าสัมผัส **NC** (ขั้ว 11-12) ของปุ่ม E-Stop ระหว่างขา 7 กับ GND (ขา 2 หรือ 4) ไม่ต้องใส่ตัวต้านทาน (ทดสอบบนโต๊ะ: จัมเปอร์ PC13 ลง GND) | ระบบถือว่ากด E-Stop ค้าง → **เบรกดึงค้าง + ตัดคันเร่ง รถขยับไม่ได้** |
| **J4 Mode_Selector** (Logger Shield, JST-XH 2 ขา ข้าง CAR_SIGNAL_IN) | สวิตช์โยกคร่อมขา 1 (PC9) – ขา 2 (GND) | ไม่ต่อ = MANUAL ตลอด (ใช้ AUTO ไม่ได้) |

> ⚠️ โฟลเดอร์ `../stm32_motorbrake_car_shield` (เวอร์ชันเก่าใน repo) ตั้ง PC13 เป็น active-low
> (ปกติ HIGH, กด = LOW). bp_version ใช้แบบ NC ลง GND (ปกติ LOW, กดหรือสายขาด = HIGH).
> ถ้ารถมีปุ่ม E-Stop ต่อ PC13 อยู่แล้ว ให้ตรวจการต่อสายให้ตรงกับ bp_version ก่อน

## พฤติกรรม

| สถานการณ์ | ผล |
|---|---|
| สวิตช์ J4 เปิด / ไม่ต่อ | **MANUAL**: เบรกไม่ขยับเอง, ไม่ขับ Curtis, ไม่รับ `/speed_enable`, `/brake_command`, ปุ่ม SW |
| สวิตช์ J4 ปิด (ไฟ D11 ติด) | **AUTO**: รับคำสั่งจาก ROS ตามปกติ |
| AUTO แล้วคนขับเหยียบคันเร่ง ≥ 50 ms | Driver takeover: ปิด speed control, ปล่อยเบรก, ค้างจนได้ `/speed_enable=0` หรือสลับสวิตช์โหมด |
| AUTO + enable แต่ `/cmd_vel` ขาด > 500 ms | ตั้งความเร็ว 0 และดึงเบรก |
| กด E-Stop (หรือสายขาด) | ดึงเบรก + ตัดคันเร่งคนขับ + ปิด AUTO (ทุกโหมด) |
| ปล่อย E-Stop ค้าง 200 ms | ปล่อยเบรก คันเร่งกลับมาให้คนขับ (AUTO ต้อง enable ใหม่) |

CAN 0x123 byte 0 เพิ่มบิต: bit3 manual, bit4 takeover, bit5 cmd_vel-timeout brake, bit6 overcurrent lockout

## วิธีแฟลช

1. STM32CubeIDE → **File → Import → Existing Projects into Workspace** → เลือกโฟลเดอร์นี้
   (ชื่อโปรเจกต์ `MotorBrake_Firmware_Car_Shield_bp_version` เปิดคู่กับโปรเจกต์เดิมได้)
2. กด **Build** (ค้อน) → ต่อ ST-Link → **Run**
3. **ไม่ต้อง/อย่า** เปิด `.ioc` แล้ว Generate Code. ถ้าถูกถาม "generate code?" ให้ตอบ **No**.
   ถ้าจำเป็นต้อง Generate ให้เช็กว่า `grep -c "BP-" Core/Src/main.c` ยังได้ 28

## ทดสอบบนโต๊ะ (ยกล้อขึ้นก่อนเสมอ) — ดูค่าใน Live Expressions

- [ ] `mode_manual` = 1 เมื่อสวิตช์ J4 เปิด, = 0 เมื่อปิด
- [ ] `estop_pc13` = 0 ปกติ, = 1 ตอนกดปุ่ม — เบรกดึง; ปล่อยปุ่มแล้วเบรกคลาย
- [ ] `curtis_pedal` = 1 ตอนเหยียบคันเร่ง (ถ้าได้ 0 ให้แก้ `PEDAL_PRESSED_LEVEL` เป็น `0u`)
- [ ] MANUAL: เหยียบคันเร่งให้ล้อหมุน — เบรกต้องไม่ขยับ
- [ ] AUTO: ส่ง `/speed_enable=true` + `/cmd_vel` แล้วหยุดส่ง `/cmd_vel` — ภายใน 0.5 s ต้องเบรก
- [ ] AUTO ระหว่างวิ่ง: เหยียบคันเร่ง — `driver_takeover` = 1, ล้อกลับมาตามคันเร่งคนขับ
