DUAL ARM ROBOTIC CONTROL SYSTEM
===============================
This folder contains the complete software ecosystem to control TWO robotic arms simultaneously from a single "Cockpit" interface.

1. LOGIC & ARCHITECTURE
-----------------------
The system relies on a "Master-Slave" logic where the PC (Master) sends simple string commands to two separate ESP32 microcontrollers (Slaves) via USB Serial.

- **The Master (Python Script):**
  - Runs a Graphical User Interface (GUI) with two distinct control panels.
  - Listens for keyboard inputs to move servos in real-time.
  - Sends commands like `s1-1500` (Servo 1 to 1500us) to the appropriate Serial Port.
  - Records the timeline of *both* arms into a single JSON file for synchronized playback.

- **The Slaves (ESP32 Firmware):**
  - Just dumb executioners. They wait for a serial command and apply the Pulse Width Modulation (PWM) signal to the requested pin.
  - They do not know about kinematics or the other arm.
  - Both arms run the IDENTICAL Firmware for simplicity.

2. FIRMWARE
-----------
We use a unified firmware for both arms. Even if Arm 2 has no magnets, it can run the magnet-enabled code without issues (the pins will just toggle high/low with nothing connected).

**Location:** `Double arm system/firmware/dual_arm_firmware.ino`
*(This is a copy of the original payload-capable firmware for convenience)*

**Upload Instructions:**
1. Open the `.ino` file in Arduino IDE.
2. Select your ESP32 Board.
3. Plug in **Arm 1 ESP32** -> Upload.
4. Plug in **Arm 2 ESP32** -> Upload.

**Wiring Map (Identical for both):**
- Servo 1 (Base): GPIO 13
- Servo 2 (Shldr): GPIO 14
- Servo 3 (Elbow): GPIO 22
- Servo 4 (Wrist): GPIO 23
- Servo 5 (Tool): GPIO 21

**Wiring Map (Arm 1 Only):**
- Magnet 1 (Relay IN1): GPIO 19
- Magnet 2 (Relay IN2): GPIO 16

3. SOFTWARE (PYTHON CONTROLLER)
-------------------------------
**File:** `manual_controller_dual.py`

**How to Start:**
1. Open terminal in this folder.
2. Run: `python manual_controller_dual.py`

**Connection Steps:**
1. Plug both ESP32s into USB.
2. In the "Arm 1" panel (Left), select the first COM port and click Connect.
   - *Tip: Test with a small move. If the wrong arm moves, disconnect and swap ports.*
3. In the "Arm 2" panel (Right), select the second COM port and connect.

**Control Scheme (Keyboard):**
*Ensure Caps Lock is OFF*

| Joint | Arm 1 (Left Hand) | Arm 2 (Right Hand) |
|:---|:---|:---|
| **Base** | Q / A | Y / H |
| **Shoulder** | W / S | U / J |
| **Elbow** | E / D | I / K |
| **Wrist** | R / F | O / L |
| **Aux** | T / G | P / ; |

4. RECORDING & PLAYBACK
-----------------------
- **Recording:** Click "Start Recording". The system captures the state of all 10 servos and 2 magnets every time a change occurs.
- **Playback:** Select a file from the dropdown and click "Play Selected". The system replays the sequence, keeping the two arms synchronized by the timestamp in the JSON file.

5. FILES STRUCTURE
------------------
/Double arm system
  |-- manual_controller_dual.py      (The Brain)
  |-- README.txt                     (This file)
  |-- firmware/
      |-- dual_arm_firmware.ino      (The Code for ESP32s)
  |-- recorded_sequences_dual/       (Where your movements are saved)
