# MotorBrake_Firmware_Car_Shield_bp_version — สิ่งที่แก้ (12 พ.ย. 2026)

คัดลอกจาก `MotorBrake_Firmware_Car_Shield` (ไม่รวม Debug/) แก้เฉพาะ `Core/Src/main.c`
ค้นหา `[BP-` ในไฟล์เพื่อดูทุกจุดที่แก้ (ต้องเจอ 28 บรรทัด — ถ้า Generate Code จาก .ioc แล้วเหลือน้อยกว่านี้ แปลว่าโค้ดหาย) ชื่อโปรเจกต์ใน CubeIDE = `MotorBrake_Firmware_Car_Shield_bp_version`
คอมไพล์และลิงก์ผ่าน (arm-none-eabi-gcc 13.2) — text 45.0 kB

| รหัส | แก้อะไร |
|---|---|
| BP-1 | อ่านสวิตช์ Mode_Selector (J4→PC9): **เปิด = MANUAL (HIGH)**, **ปิดลง GND = AUTO (LED D11 ติด)**. MANUAL: ไม่ขยับเบรก (ยกเว้น E-Stop PC13), ไม่ขับ Curtis, ไม่รับ /speed_enable, /brake_command, ปุ่ม PC1. สลับโหมดเมื่อไหร่ ล้างคำสั่ง auto ทั้งหมด |
| BP-1/3 | auto-brake ทำงานเฉพาะ AUTO + speed_control_enable + cmd_vel สด (เดิมทำงานแม้ขับแมนนวล) |
| BP-2 | Driver takeover: AUTO แล้วคนขับเหยียบคันเร่ง ≥50 ms → ปิด speed control, ปล่อยเบรก, latch จนกว่าจะได้ /speed_enable=0 หรือสลับสวิตช์โหมด |
| BP-3 | cmd_vel ขาด >500 ms ขณะ AUTO+enable → target 0 และเบรกค้างไว้ (ทำบน STM32 เอง) |
| BP-4 | PC13 ใส่ pull-down (ขาไม่ได้ต่อ เดิมลอยแล้วสั่งเบรกเองได้); E-Stop ใช้มุม stop ที่คาลิเบรต |
| BP-5 | Overcurrent แล้วห้ามเบรกซ้ำ 2 s (กันรีเลย์ตัด-ต่อรัวๆ) |
| BP-6 | เตะ watchdog (WDI) จาก main loop แทน ISR |
| BP-7 | E-Stop ที่ PC13: ปุ่มหัวเห็ดหน้าสัมผัส NC ต่อ PC13–GND (pull-up ภายใน). กด/สายขาด = เบรกดึง + ตัดคันเร่งคนขับ (mode relay ON, Curtis outputs 0) + ปิด AUTO; ปล่อยปุ่มค้าง 200 ms = เบรกปล่อย คันเร่งกลับมา. **ยังไม่ต่อปุ่ม = เบรกค้าง** (ทดสอบบนโต๊ะให้จัมเปอร์ PC13 ลง GND). CN3 EMER บนบอร์ดล่างให้เสียบจัมเปอร์ค้างไว้ |
| CAN 0x123 byte0 | bit3 manual, bit4 takeover, bit5 timeout-brake, bit6 OC lockout |

ต้องเช็กบนโต๊ะก่อนวิ่งจริง: ระดับ `curtis_pedal` ตอนเหยียบ = 1 หรือไม่ (`PEDAL_PRESSED_LEVEL`), และ `mode_manual` ตรงกับตำแหน่งสวิตช์
