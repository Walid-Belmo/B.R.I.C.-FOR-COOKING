# ==========================================
# ROBOT ARM GEOMETRIC CONFIGURATION
# ==========================================
# Units: Millimeters (mm)
# Derived from physical measurements and STL analysis.

# 1. Base (Origin) -> Shoulder (J2)
# Offsets from the Center of the Base (J1 axis) to the Center of the Shoulder (J2 axis).
# Interpreted as a fixed translation inside the J1 rotating frame.
J1_TO_J2_X = 1.70
J1_TO_J2_Y = 14.06
J1_TO_J2_Z = 97.48

# 2. Shoulder (J2) -> Elbow (J3)
# Straight link length.
# Assuming this link aligns with the Z-axis of the previous frame when angle is 0 (Vertical).
J2_TO_J3_LEN = 120.00

# 3. Elbow (J3) -> Wrist (J4)
# Offsets from the Elbow Axis center to the Wrist Axis center.
# J3 rotates around X. These are in the J3 frame.
# Assuming Z is the "Length" of the arm.
J3_TO_J4_X = 11.49
J3_TO_J4_Y = 5.31
J3_TO_J4_Z = 89.75

# 4. Wrist (J4) -> Tool Tip
# Offsets from the Wrist Axis center to the Tool Tip.
# J4 rotates around Z. These are in the J4 frame.
J4_TO_TIP_X = -14.00
J4_TO_TIP_Y = -4.64
J4_TO_TIP_Z = 50.00

# ==========================================
# SERVO CONFIGURATION
# ==========================================
# Servos J1-J3: TD-8120MG (270 degree range)
#   Scale: 270 / 2000 = 0.135 degrees per microsecond
# Servo J4: MG90S (180 degree range)
#   Scale: 180 / 2000 = 0.09 degrees per microsecond

DEG_PER_US = [0.135, 0.135, 0.135, 0.09]

# SERVO DIRECTIONS & OFFSETS
# --------------------------
# Adjust these to match your specific assembly.
# DIRECTION: 1 for Normal, -1 for Inverted (Reversed)
# OFFSET: Angle in degrees to add to the calculated value.
#         Use this if 1500us (Neutral) is NOT pointing Straight Up (0 degrees).

# Hypothesis based on your feedback (Z=4 vs Z=247):
# J2 (Shoulder): Likely Inverted (High pulse = Down).
# J3 (Elbow): Likely Inverted or Offset.

JOINT_DIRECTIONS = [-1, 1, -1, -1]  # Updated by calibrate.py
JOINT_OFFSETS    = [0.0, 4.3, 0.0, 0.0]  # J2 offset: Compensates for geometric/mounting misalignment
                                          # When GUI shows 85.7°, physical angle is 90° -> need +4.3° correction
                                          # Verified: -90° position is accurate, +90° needs correction

# Calibration Trims (Pulse Width Offsets)
# Used to zero the robot. 
# Target: When Pulse = 1500 + Trim, the joint angle should be logically 0 (Vertical).
# These will be adjustable in the GUI.
DEFAULT_TRIMS = [0, 29, 0, 0]  # J2 trim: 29us shift to achieve true vertical (0°) position
