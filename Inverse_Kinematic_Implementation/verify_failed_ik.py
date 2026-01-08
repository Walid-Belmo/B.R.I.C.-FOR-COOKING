"""
Check what position the failed IK pulses actually produce
"""
import kinematics

print("="*70)
print("VERIFY FAILED IK RESULT")
print("="*70)

# Target
target_x = -0.81
target_y = -195.07
target_z = 1.22

# Failed IK result
failed_pulses = [1537, 2500, 1980, 561]

print(f"\nTarget: ({target_x}, {target_y}, {target_z})")
print(f"Failed IK Pulses: {failed_pulses}")

# Compute FK
fk = kinematics.forward_kinematics(failed_pulses, kinematics.cfg.DEFAULT_TRIMS)

print(f"\nForward Kinematics of failed pulses:")
print(f"  Position: ({fk['x']:.2f}, {fk['y']:.2f}, {fk['z']:.2f})")
print(f"  Angles: q1={fk['angles'][0]:.1f}°, q2={fk['angles'][1]:.1f}°, q3={fk['angles'][2]:.1f}°, q4={fk['angles'][3]:.1f}°")

error = ((fk['x'] - target_x)**2 + (fk['y'] - target_y)**2 + (fk['z'] - target_z)**2)**0.5
print(f"\nError: {error:.2f}mm")

print("\nNote: J2 hit limit (2500) and J4 hit limit (561)")
print("This suggests the target might be at the edge of reachable workspace")

print("="*70)
