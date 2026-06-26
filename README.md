# A.R.I.S. - Autonomous Rescue & Inspection System

> **Innovate ECE 2026 · Robotics & Autonomous Systems Thread · Georgia Institute of Technology**  
> *Saksham Bansal*

A.R.I.S. is an autonomous ground rover designed to enter structurally compromised disaster environments ahead of human personnel. It perceives its surroundings through a multi-modal sensor array, makes decisions using a non-blocking Finite State Machine (FSM), and streams real-time telemetry to an operator dashboard.




## Demo

---<img width="991" height="599" alt="Screenshot 2026-03-30 at 9 51 57 PM" src="https://github.com/user-attachments/assets/46ebeda9-5801-4d2f-adf5-938d465f5063" />

*The live WebSocket dashboard showing IMU heading, sonar clearance, and hazard air quality.*

---

## How It Works

A.R.I.S. operates on a **Sense → Plan → Act** pipeline across three layers:

| Layer | Components |
|---|---|
| **Perception** | HC-SR04 Ultrasonic · GY-521 MPU6050 IMU · MQ-6 Gas Sensor |
| **Processing** | ESP32 FSM + P-Controller (heading correction) |
| **Action & Telemetry** | L298N Motor Driver · 4WD Drivetrain · Async WebSocket Dashboard |

### FSM States

```
EXPLORING  ──(obstacle < 25cm)──▶  TURNING  ──(path clear > 35cm)──▶  EXPLORING
     │                                  │
     └──(gas ADC > threshold)──▶  HAZARD DETECTED (halt & pulse)
```

- **EXPLORING** - Default state. Drives forward with closed-loop heading correction. A proportional controller reads Z-axis gyroscope data and applies a corrective PWM differential across the left/right motor channels to maintain a straight bearing.
- **TURNING** - Triggered when an obstacle is detected within 25 cm. Executes a closed-loop 90° point-turn using gyroscope threshold-crossing feedback. Dynamically monitors clearance and immediately resumes exploration if the path clears (> 35 cm), preventing infinite spin traps.
- **HAZARD DETECTED** - Triggered when the MQ-6 gas ADC reading exceeds the calibrated threshold. Halts all motion and pulses the motors as a physical alert signal.

---

## Bill of Materials

| Component | Part / Specification |
|---|---|
| Microcontroller | ESP32-S Development Board (38-pin or 30-pin) |
| Motor Driver | L298N Dual H-Bridge |
| Chassis | Standard 4WD robot chassis kit (acrylic, 4× DC gear motors, 4× wheels) |
| Inertial Sensor | GY-521 (MPU6050 6-axis IMU) |
| Distance Sensor | HC-SR04 Ultrasonic Sensor |
| Gas Sensor | MQ-6 Analog Gas Sensor |
| Power | 5V USB Power Bank (logic) + 6V AA battery pack (motors) |
| Hardware | Jumper wires (M-F and M-M), double-sided foam tape |

---

## Wiring Reference

### Motor Control (L298N → ESP32)

| L298N Pin | ESP32 GPIO |
|---|---|
| ENA (Left Speed PWM) | GPIO 13 |
| IN1 (Left Direction A) | GPIO 33 |
| IN2 (Left Direction B) | GPIO 14 |
| IN3 (Right Direction A) | GPIO 27 |
| IN4 (Right Direction B) | GPIO 26 |
| ENB (Right Speed PWM) | GPIO 25 |

### Sensors

| Sensor / Pin | ESP32 GPIO |
|---|---|
| HC-SR04 TRIG | GPIO 32 |
| HC-SR04 ECHO | GPIO 18 |
| MPU6050 SCL | GPIO 22 |
| MPU6050 SDA | GPIO 21 |
| MQ-6 AOUT | GPIO 36 (ADC1 — required for Wi-Fi) |

> **⚠️ Power Isolation Required:** Power the ESP32 from a 5V USB bank and the L298N from the 6V battery pack separately. Share only GND. **Leave the L298N's onboard 5V pin completely empty** - bridging it to the ESP32 will cause brownouts.

> **⚠️ GPIO Warning:** Do not map pins by physical position. Use the GPIO numbers printed on the ESP32 silkscreen to avoid flash memory conflicts.

---

## Software Setup

### 1. Install Arduino IDE & ESP32 Support

1. Download [Arduino IDE 2.x](https://www.arduino.cc/en/software).
2. Go to **File → Preferences** and add the following to *Additional Boards Manager URLs*:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Go to **Tools → Board → Boards Manager**, search for `esp32`, and install the package by **Espressif Systems**.

### 2. Install Required Libraries

Via **Sketch → Include Library → Manage Libraries**:
- `Adafruit MPU6050` (installs `Adafruit Unified Sensor` as a dependency)

Manually via **Sketch → Include Library → Add .ZIP Library**:
- [ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)
- [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)

### 3. Flash the Firmware

1. Clone this repo and open `ARIS.ino` in Arduino IDE.
2. Go to **Tools → Board** and select `ESP32 Dev Module`. Set the correct **Port**.
3. Click **Upload**.

> **Tip:** If the console hangs at `Connecting...`, hold the physical **BOOT** button on the ESP32 until the upload percentage starts.

---

## Operation

### Boot Sequence
1. Plug the USB power bank into the ESP32, then connect the 6V battery to the L298N.
2. Place the rover on a flat surface and **do not move it for 3 seconds**. The MPU6050 samples 200 gyroscope readings on startup to calibrate its Z-axis drift offset. Moving the rover during this window corrupts the heading baseline.

### Accessing the Command Center
1. On any Wi-Fi device, connect to:
   - **Network:** `ARIS_Telemetry`
   - **Password:** `innovate_ece`
2. Open a browser and navigate to `http://192.168.4.1`.
3. The dashboard streams live IMU heading, sonar clearance, and gas level **4× per second** via WebSocket.

### Quick Tests
- **Drive Test:** Block the ultrasonic sensor with your hand at < 20 cm. The state badge will switch to `EVASIVE MANEUVER` and the rover will execute a 90° point-turn.
- **Hazard Test:** Wave an uncapped dry-erase marker or an isopropyl alcohol swab beneath the MQ-6 sensor. The badge will flash `HAZARD DETECTED!` and the motors will halt and pulse.

---

## Key Implementation Notes

- **ADC1 only for gas sensor** - ADC2 is disabled during Wi-Fi operation on ESP32. GPIO 36 (ADC1) is required.
- **Non-blocking FSM** - The main loop never calls `delay()` during navigation states, keeping WebSocket telemetry responsive.
- **Inverted gyro math** - The yaw integration uses `-=` (not `+=`) to correct for the gyroscope's positive feedback orientation on this chassis.
- **Gas threshold** - Default is `1800` on the 0–4095 ADC scale. Tune `gasThreshold` in the firmware after a 10-minute warm-up period for your environment.

---

<img width="640" height="546" alt="55236679976_376ef51f6d_z" src="https://github.com/user-attachments/assets/09c17736-f2ce-4644-ad5f-e9254c8ce2f2" />
<img width="639" height="593" alt="55235773642_5f0d156675_z" src="https://github.com/user-attachments/assets/bb56f377-d260-40e2-8874-12b94b5e6f71" />


## Project Structure

```
ARIS/
├── ARIS.ino                        # Main firmware (FSM, sensors, WebSocket server)
├── ARIS_UserGuide_Final.pdf        # Full assembly & operation manual
├── 3MT_Presentation_Slides.pdf     # 3-Minute Thesis presentation slides
└── README.md
```

---

## License

This project was developed for Innovate ECE 2026 at Georgia Institute of Technology. Feel free to use, modify, and build on it for educational and non-commercial purposes.
