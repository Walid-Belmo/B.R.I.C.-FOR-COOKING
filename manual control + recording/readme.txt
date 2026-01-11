===========================================================
ROBOT ARM MANUAL CONTROLLER & SEQUENCE RECORDER - README
===========================================================

1. OVERVIEW
-----------
This tool allows you to manually control a 5-DOF robot arm, record your movements into sequences, and replay them back in a specific order. It is designed to help you "teach" the robot complex tasks (like cooking motions) by demonstration.

2. FEATURES
-----------
- **Manual Control**: Move servos using Keyboard, Sliders, or Direct Value Entry.
- **Recording**: Capture smooth movements in real-time.
- **Auto-Save**: Automatically handles file naming to prevent overwriting (e.g., "stir_1", "stir_2").
- **Playlist System**: String together multiple recorded sequences (e.g., [Pick Lid] -> [Stir] -> [Place Lid]).
- **Safety**: 
    - Strict position verification ensuring sequences match perfectly.
    - "Reset to 1500" button for quick safe-homing.

3. HARDWARE & SETUP
-------------------
- **Firmware**: Ensure the ESP32 is running `ESP32_FIRMWARE/simple_servo_control.ino`.
- **Wiring**: Servos connected to pins [13, 14, 22, 23, 21].
- **Connection**: Connect ESP32 via USB. The tool auto-detects COM ports.
- **Baud Rate**: 115200

4. MANUAL CONTROL METHODS
-------------------------
A. KEYBOARD SHORTCUTS
   - Q / A : Servo 1 (Base)
   - W / S : Servo 2 (Shoulder)
   - E / D : Servo 3 (Elbow)
   - R / F : Servo 4 (Wrist)
   - T / G : Servo 5 (Aux/Gripper)

B. SLIDERS
   - Drag the horizontal sliders for coarse control. Great for large movements.

C. DIRECT ENTRY
   - Type a value (500-2500) in the text box and press Enter (or click "Set").
   - Useful for returning to exact known positions.

D. RESET
   - Click "Reset All (1500)" to center all servos immediately.

5. RECORDING A SEQUENCE
-----------------------
1. Enter a name in the "Name" field (e.g., "mix_soup").
2. Click **Start Recording**.
3. Perform the movements using any control method. Every move is captured.
4. Click **Stop Recording**.
5. The sequence is saved to `recorded_sequences/mix_soup.json`.

*Note: If "mix_soup" already exists, it will save as "mix_soup_1" automatically.*

6. PLAYBACK & PLAYLISTS
-----------------------
1. **Available Sequences**: Your saved files appear in the left list.
2. **Build Playlist**: Select a file and click "Add >>" to move it to the execution playlist on the right.
3. **Reorder**: Use "Up" and "Down" buttons to arrange the order of tasks.
4. **Run**: Click "Safe Play Playlist".

*CRITICAL SAFETY RULE*:
The robot checks if its *current physical position* matches the *start position* of the sequence you are trying to play.
- If they match (within tolerance): The sequence plays.
- If they DO NOT match: The robot STOPS and warns you.

This ensures you don't accidentally run a sequence starting from the wrong location, which could break the arm or the cooking pot.

7. FILE STRUCTURE
-----------------
manual_controller.py      # Main application
recorded_sequences/       # Folder where .json movement files are stored
readme.txt                # This file
