import numpy as np
import kinematics
import kinematics_config as cfg

def calculate_pulse_for_angle(target_angle, joint_idx):
    """
    Reverse calculates the pulse needed to hit a specific angle 
    based on current finding in kinematics_config.
    """
    # angle = (delta * scale * dir) + offset
    # angle - offset = delta * scale * dir
    # (angle - offset) / (scale * dir) = delta
    # pulse = neutral + delta
    
    offset = cfg.JOINT_OFFSETS[joint_idx]
    direction = cfg.JOINT_DIRECTIONS[joint_idx]
    scale = cfg.DEG_PER_US
    neutral = 1500 + cfg.DEFAULT_TRIMS[joint_idx]
    
    delta = (target_angle - offset) / (scale * direction)
    return int(neutral + delta)

def print_scenario(name, pulses):
    print(f"\n--- SCENARIO: {name} ---")
    print(f"Sending Pulses: {pulses}")
    
    # Calculate Forward Kinematics
    res = kinematics.forward_kinematics(pulses)
    
    # Calculate Cylindrical Coordinates (easier to measure with a ruler)
    # Radius = sqrt(x^2 + y^2) (Distance from center pole)
    # Height = z (Distance from table/base)
    r = np.sqrt(res['x']**2 + res['y']**2)
    
    print(f"Calculated Angles: {[round(a, 2) for a in res['angles']]}")
    print(f"PREDICTED TIP POSITION:")
    print(f"  X: {res['x']:.2f} mm")
    print(f"  Y: {res['y']:.2f} mm")
    print(f"  Z: {res['z']:.2f} mm  <-- CHECK THIS HEIGHT")
    print(f"  Radius (Extension): {r:.2f} mm <-- CHECK THIS DISTANCE FROM CENTER")

print("="*60)
print("DEBUG PLAN: PREDICTED VALUES")
print("="*60)

# SCENARIO 1: NEUTRAL (As currently configured)
# The user wants to check if robot height makes sense at rest.
pulses_neutral = [1500, 1500, 1500, 1500]
print_scenario("1. NEUTRAL COMMAND (1500 all)", pulses_neutral)

# SCENARIO 2: HORIZONTAL SHOULDER
# We want Servo 2 at 90 degrees. Others at 0 degrees.
# Note: 0 degrees might not be 1500 pulse if trims/offsets are set.
# Let's calculate the pulse for exactly 0, 90, 0, 0 degrees.
p1_0 = calculate_pulse_for_angle(0, 0)
p2_90 = calculate_pulse_for_angle(90, 1) # 90 degrees for shoulder
p3_0 = calculate_pulse_for_angle(0, 2)
p4_0 = calculate_pulse_for_angle(0, 3)

pulses_horizontal = [p1_0, p2_90, p3_0, p4_0]
print_scenario("2. HORIZONTAL SHOULDER (Target: J2=90°, Others=0°)", pulses_horizontal)

print("\n" + "="*60)
print("INSTRUCTIONS:")
print("1. Measure the real height of the robot tip in Scenario 1.")
print("2. Send the pulses from Scenario 2 to the robot.")
print("3. Check if the arm is physically horizontal.")
print("4. Measure the height of the tip in Scenario 2.") 
print("   It should be approx equal to the Shoulder Axis Height.")
