DUAL ARM MANUAL CONTROL SYSTEM
==============================

This folder contains the software to control TWO robotic arms simultaneously from a single interface.

FILES
-----
1. manual_controller_dual.py
   - The main Python application.
   - Run this to open the GUI.

HOW TO USE
----------
1. Connect both ESP32s to your computer via USB.
2. Run `python manual_controller_dual.py`
3. In the GUI:
   - Select the COM port for Arm 1 (Left Panel, with Magnets) and click Connect.
   - Select the COM port for Arm 2 (Right Panel, No Magnets) and click Connect.
   
CONTROLS
--------
You can use the GUI sliders OR the keyboard.

**Warning:** Ensure "Caps Lock" is OFF.

| Joint | ARM 1 (Left Hand) | ARM 2 (Right Hand) |
| :--- | :--- | :--- |
| **Base** | Q / A | Y / H |
| **Shoulder** | W / S | U / J |
| **Elbow** | E / D | I / K |
| **Wrist** | R / F | O / L |
| **Aux/Tool** | T / G | P / ; |

FEATURES
--------
- **Dual Recording:** When you hit "Start Recording", it captures the movements of BOTH arms into a single JSON timeline.
- **Playback:** You can replay these dual-arm sequences.
- **Magnets:** Only Arm 1 has magnet controls (as requested).

FIRMWARE
--------
- **Arm 1 (with Magnets):** Use `../manual control + recording/firmware_with_magnets/controlled_arm_with_magnets.ino`
- **Arm 2 (No Magnets):** You can use the same firmware (and ignore magnet pins) OR use the original `../ESP32_FIRMWARE/simple_servo_control.ino` if the pins match (13, 14, 22, 23, 21).

PINOUT REMINDER
---------------
Servo 1: 13
Servo 2: 14
Servo 3: 22
Servo 4: 23
Servo 5: 21
