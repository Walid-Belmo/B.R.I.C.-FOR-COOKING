# TESTING GUIDE - Forward Kinematics Verification

## Step 1: Activate Your Virtual Environment

```bash
cd C:\Users\iceoc\Documents\cooker_mini_bras\PYTHON_CONTROL
# Activate your venv (adjust path as needed)
# Example: .venv\Scripts\activate
```

## Step 2: Run the Test Suite (No Hardware Required)

This verifies the math is working:

```bash
python test_kinematics.py
```

**Expected Output:** 10 test cases showing positions for different joint angles.

## Step 3: Connect Your Robot

1. Plug in the ESP32 via USB
2. Note which COM port it connects to (e.g., COM3, COM5)
3. Make sure the ESP32 firmware is already uploaded

## Step 4: Launch the GUI

```bash
python driver_gui.py
```

**What you should see:**
- Window titled "4-DOF Robot Arm Control & Kinematics"
- Serial connection dropdown
- 4 servo sliders (500-2500us range)
- Forward kinematics display showing X, Y, Z position
- Log window at the bottom

## Step 5: Connect to Robot

1. Click the "Refresh Ports" button
2. Select your ESP32 COM port from dropdown
3. Click "Connect"
4. **Check log window:** Should say "Connected to COMx"

## Step 6: Initial Safety Check (CRITICAL!)

**Before moving servos, verify the robot is in a safe position:**

1. All sliders should be at 1500us (neutral)
2. The GUI should show approximately:
   - X: -0.81 mm
   - Y: -40.18 mm
   - Z: 351.51 mm
   - Angles: q1=0.0°, q2=12.2°, q3=0.0°, q4=0.0°

3. **Physically check:** Is the robot standing mostly upright?

## Step 7: Test Individual Joints (One at a Time!)

### Test J1 (Base Rotation)

1. **Slowly** move Servo 1 slider from 1500 → 1600 (small increment)
2. **Observe physical robot:**
   - Should rotate clockwise (viewed from above)
   - If it rotates counter-clockwise → Joint direction is WRONG
3. **Observe GUI:**
   - q1 angle should increase (positive)
   - X and Y should change, Z should stay constant
4. Move back to 1500

**If direction is WRONG:**
- Open `kinematics_config.py`
- Line 53: Change `JOINT_DIRECTIONS = [1, 1, 1, 1]` to `[-1, 1, 1, 1]`
- Restart the GUI

### Test J2 (Shoulder Pitch)

1. Move Servo 2 slider from 1500 → 1600
2. **Observe physical robot:**
   - Should pitch forward (arm moving away from vertical)
   - If it pitches backward → Joint direction is WRONG
3. **Observe GUI:**
   - q2 angle should increase
   - Y should decrease (forward), Z should decrease (down)
4. Move back to 1500

**If direction is WRONG:**
- Change `JOINT_DIRECTIONS = [1, 1, 1, 1]` to `[1, -1, 1, 1]`

### Test J3 (Elbow Bend)

1. Move Servo 3 slider from 1500 → 1600
2. **Observe physical robot:**
   - Should bend forward (forearm moving relative to upper arm)
   - If it bends backward → Joint direction is WRONG
3. **Observe GUI:**
   - q3 angle should increase
   - Y and Z should change
4. Move back to 1500

**If direction is WRONG:**
- Change `JOINT_DIRECTIONS = [1, 1, 1, 1]` to `[1, 1, -1, 1]`

### Test J4 (Wrist Rotation)

1. Move Servo 4 slider from 1500 → 1600
2. **Observe physical robot:**
   - Should rotate around its own axis
   - The tool tip should orbit in a circle (due to offset)
3. **Observe GUI:**
   - q4 angle should increase
   - X, Y, Z should change slightly
4. Move back to 1500

**If direction is WRONG:**
- Change `JOINT_DIRECTIONS = [1, 1, 1, 1]` to `[1, 1, 1, -1]`

## Step 8: Verify Forward Kinematics Accuracy

### Test Position 1: Neutral (All 1500us)

1. Set all servos to 1500us
2. **Measure with ruler:**
   - X: Distance from base center in forward direction
   - Y: Distance from base center sideways
   - Z: Height from table/base to tool tip
3. **Compare to GUI display**
4. **Expected:** Should match within ±10mm

### Test Position 2: J1 at 90° (approx 2167us)

Calculation: 90° / 0.135 = 667us offset → 1500 + 667 = 2167us

1. Move Servo 1 to 2167us
2. Robot should be rotated 90° from neutral
3. **Measure X, Y, Z with ruler**
4. **Compare to GUI**

### Test Position 3: J2 Forward (1833us)

1. Move Servo 2 to 1833us (about +45° pitch forward)
2. Arm should be reaching forward
3. **Measure Z height** - should be lower than neutral
4. **Compare to GUI**

## Step 9: Check for Systematic Errors

If measurements are consistently off by a fixed amount:

### Error Type A: Constant Offset in Angle
**Symptom:** When slider is at 1500us, the joint isn't at 0° physically

**Fix:** Adjust `DEFAULT_TRIMS` in `kinematics_config.py` line 60

Example: If J2 at 1500us is actually pointing 10° backward:
```python
DEFAULT_TRIMS = [0, -74, 0, 0]  # Was [0, -60, 0, 0], subtract 10°/0.135 ≈ 74us
```

### Error Type B: Position Always Off by Same Direction
**Symptom:** Calculated position is always 20mm too high

**Fix:** Check your measurements in `kinematics_config.py` lines 10-32
- Likely one of the Z offsets is wrong

### Error Type C: Position Error Scales with Movement
**Symptom:** Small movements are close, large movements are way off

**Fix:** Joint direction is inverted, or servo scale factor is wrong

## Step 10: Use the Calibration Tool

Once basic movement works, run the calibration tool:

```bash
python calibrate.py
```

This helps you find precise `JOINT_DIRECTIONS` and `JOINT_OFFSETS` values.

## Troubleshooting

### Robot doesn't move when I move sliders
- Check serial connection (log should show "Servo X set to YYYY")
- Check ESP32 is running the correct firmware
- Try manually sending: Click Connect, type in log area doesn't work - use Arduino Serial Monitor to test `s1-1600`

### GUI crashes when I move sliders
- Check numpy is installed in your venv
- Check kinematics.py and kinematics_config.py are in the same folder

### Position calculation seems random
- One or more JOINT_DIRECTIONS is wrong
- Go back to Step 7 and test each joint individually

### Position is close but not exact
- This is normal! Calibration needed
- Use Step 9 to fine-tune TRIMS and OFFSETS

## Expected Accuracy

After calibration, you should achieve:
- **±5mm** accuracy for positions within 200mm reach
- **±10mm** accuracy for full reach positions
- **±2-3°** accuracy for joint angles

## Safety Notes

- **NEVER** move sliders rapidly - servos can draw high current
- **ALWAYS** test one joint at a time first
- **WATCH** for servo buzzing (fighting against load) - reduce speed
- **STOP** if anything looks mechanically wrong

## Next Steps After Testing

Once FK is verified:
1. Record positions in a log file (GUI does this automatically)
2. Create a calibration report comparing measured vs calculated
3. Fine-tune configuration values
4. Start working on Inverse Kinematics (IK)

---

Good luck! Report back with:
- Which joints (if any) needed direction flipping
- Measured vs calculated positions
- Any weird behavior
