"""
Test script for safety features
Tests collision detection and safe move planning
"""
import kinematics

print("="*60)
print("SAFETY FEATURES TEST")
print("="*60)

# Test 1: Home position
print("\n1. Home Position Test:")
home = kinematics.get_home_position_pulses()
print(f"   Home pulses: {home}")

# Test 2: Collision detection with base
print("\n2. Collision Detection Test:")
test_points = [
    (0, 0, 30, "Inside base cylinder"),
    (50, 0, 30, "Outside base cylinder"),
    (0, 0, 100, "Above base cylinder"),
    (20, 20, 30, "Near base edge"),
]

for x, y, z, desc in test_points:
    collision = kinematics.check_collision_with_base(x, y, z)
    status = "⚠ COLLISION" if collision else "✓ Safe"
    print(f"   ({x:4.0f}, {y:4.0f}, {z:4.0f}) - {desc:25s} : {status}")

# Test 3: Safe move planning
print("\n3. Safe Move Planning Test:")
current_pulses = [1500, 1600, 1500, 1500]
test_targets = [
    (0, 100, 150, "Safe target forward"),
    (0, 0, 30, "Target inside base - should BLOCK"),
    (0, 150, 200, "Target above workspace"),
]

for x, y, z, desc in test_targets:
    print(f"\n   Target: ({x}, {y}, {z}) - {desc}")
    result = kinematics.safe_move_to_target(x, y, z, current_pulses)
    
    if result['success']:
        print(f"   ✓ APPROVED - Path validated")
        print(f"   Steps: Current → Home → Target")
    else:
        print(f"   ✗ BLOCKED - {result['error']}")

print("\n" + "="*60)
print("Safety tests complete!")
print("="*60)
