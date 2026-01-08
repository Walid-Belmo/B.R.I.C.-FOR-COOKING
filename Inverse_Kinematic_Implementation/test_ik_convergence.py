"""
Test IK convergence for the specific case that failed
Target: (-0.81, -195.07, 1.22)
"""
import kinematics

print("="*70)
print("IK CONVERGENCE TEST")
print("="*70)

# Target from your screenshot
target_x = -0.81
target_y = -195.07
target_z = 1.22

print(f"\nTarget Position: ({target_x}, {target_y}, {target_z})")

# Get home position
home_pulses = kinematics.get_home_position_pulses()
print(f"Home Pulses: {home_pulses}")

# Test IK with increased iterations
print("\n" + "-"*70)
print("Running IK from HOME position...")
print("-"*70)

result = kinematics.inverse_kinematics_numerical(
    target_x, target_y, target_z, 
    current_pulses=home_pulses,
    max_iter=200,
    tolerance=1.0
)

print(f"\nResult:")
print(f"  Success: {result['success']}")
print(f"  Error: {result['error']:.2f}mm")
print(f"  Iterations: {result['iterations']}")
print(f"  Final Pulses: {result['pulses']}")

if result['success']:
    print("\n✓ IK CONVERGED!")
    
    # Verify by computing FK
    fk = kinematics.forward_kinematics(result['pulses'], kinematics.cfg.DEFAULT_TRIMS)
    print(f"\nVerification (Forward Kinematics):")
    print(f"  Computed Position: ({fk['x']:.2f}, {fk['y']:.2f}, {fk['z']:.2f})")
    print(f"  Target Position:   ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")
    print(f"  Error: {result['error']:.2f}mm")
else:
    print("\n✗ IK FAILED TO CONVERGE")
    print(f"  Final error: {result['error']:.2f}mm after {result['iterations']} iterations")

print("\n" + "="*70)
