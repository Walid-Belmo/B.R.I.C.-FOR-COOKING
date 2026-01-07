"""
Test script for forward kinematics verification
Run this to verify the FK calculations before testing with the robot
"""
import kinematics

def test_position(name, pulses, trims=None):
    """Test a specific servo position"""
    result = kinematics.forward_kinematics(pulses, trims)
    print(f"\n{name}:")
    print(f"  Pulses: {pulses}")
    print(f"  Angles: q1={result['angles'][0]:.1f}°, q2={result['angles'][1]:.1f}°, q3={result['angles'][2]:.1f}°, q4={result['angles'][3]:.1f}°")
    print(f"  Position: X={result['x']:.2f} mm, Y={result['y']:.2f} mm, Z={result['z']:.2f} mm")
    return result

print("="*60)
print("FORWARD KINEMATICS TEST CASES")
print("="*60)

# Test 1: All servos neutral (1500us)
test_position("Test 1: All Neutral (1500us)", [1500, 1500, 1500, 1500])

# Test 2: J1 rotated +45deg (approx 1833us for 270deg servo)
# 45deg / 0.135 = 333us offset
test_position("Test 2: J1 +45deg", [1833, 1500, 1500, 1500])

# Test 3: J1 rotated -45deg
test_position("Test 3: J1 -45deg", [1167, 1500, 1500, 1500])

# Test 4: J2 pitched forward +30deg
# 30deg / 0.135 = 222us offset (but J2 has -60us trim)
test_position("Test 4: J2 +30deg", [1500, 1722, 1500, 1500])

# Test 5: J2 pitched backward -30deg
test_position("Test 5: J2 -30deg", [1500, 1278, 1500, 1500])

# Test 6: J3 bent +45deg
test_position("Test 6: J3 +45deg", [1500, 1500, 1833, 1500])

# Test 7: J4 rotated +45deg
test_position("Test 7: J4 +45deg", [1500, 1500, 1500, 1833])

# Test 8: Combined movement
test_position("Test 8: Combined (all +20deg)", [1648, 1648, 1648, 1648])

# Test 9: Reach forward (J2 and J3 both +45deg)
test_position("Test 9: Reach Forward", [1500, 1833, 1833, 1500])

# Test 10: Extreme high position
test_position("Test 10: Maximum Height", [1500, 1500, 500, 1500])

print("\n" + "="*60)
print("TESTS COMPLETE")
print("="*60)
print("\nNOTE: These are theoretical calculations.")
print("Physical testing is required to verify accuracy.")
print("Use calibrate.py to fine-tune JOINT_DIRECTIONS and JOINT_OFFSETS.")
