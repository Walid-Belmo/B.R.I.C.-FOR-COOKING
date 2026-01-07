"""
Manual verification of forward kinematics algorithm
This traces through the transformation matrices step-by-step
"""
import numpy as np
import kinematics_config as cfg
import kinematics

print('='*60)
print('FORWARD KINEMATICS MATHEMATICAL VERIFICATION')
print('='*60)
print()

# Test neutral position
pulses = [1500, 1500, 1500, 1500]
trims = cfg.DEFAULT_TRIMS

print('INPUT:')
print(f'  Pulses: {pulses}')
print(f'  Trims: {trims}')
print()

# Convert to angles
angles = []
for i in range(4):
    ang = kinematics.pulse_to_angle(pulses[i], trims[i], i)
    angles.append(ang)

q1, q2, q3, q4 = angles
print('ANGLES:')
print(f'  q1 = {q1:.2f}° (Base yaw)')
print(f'  q2 = {q2:.2f}° (Shoulder pitch)')
print(f'  q3 = {q3:.2f}° (Elbow pitch)')
print(f'  q4 = {q4:.2f}° (Wrist yaw)')
print()

print('TRANSFORMATION CHAIN:')
print('-'*60)

# Build full transformation manually
print('1. J1 rotation (Z-axis by q1)')
R1 = kinematics.get_rotation_matrix('z', q1)
M = R1
pos = M[:3, 3]
print(f'   Position after J1: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

print('2. Translate to shoulder, then J2 rotation (X-axis by q2)')
T = kinematics.get_translation_matrix(cfg.J1_TO_J2_X, cfg.J1_TO_J2_Y, cfg.J1_TO_J2_Z)
R2 = kinematics.get_rotation_matrix('x', q2)
M = M @ T @ R2
pos = M[:3, 3]
print(f'   Position after J2: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

print('3. Translate to elbow, then J3 rotation (X-axis by q3)')
T = kinematics.get_translation_matrix(0, 0, cfg.J2_TO_J3_LEN)
R3 = kinematics.get_rotation_matrix('x', q3)
M = M @ T @ R3
pos = M[:3, 3]
print(f'   Position after J3: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

print('4. Translate to wrist, then J4 rotation (Z-axis by q4)')
T = kinematics.get_translation_matrix(cfg.J3_TO_J4_X, cfg.J3_TO_J4_Y, cfg.J3_TO_J4_Z)
R4 = kinematics.get_rotation_matrix('z', q4)
M = M @ T @ R4
pos = M[:3, 3]
print(f'   Position after J4: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')

print('5. Translate to tool tip')
T = kinematics.get_translation_matrix(cfg.J4_TO_TIP_X, cfg.J4_TO_TIP_Y, cfg.J4_TO_TIP_Z)
M = M @ T
pos_manual = M[:3, 3]
print(f'   Final Position: ({pos_manual[0]:.2f}, {pos_manual[1]:.2f}, {pos_manual[2]:.2f})')

print()
print('-'*60)
print('VERIFICATION:')
print('-'*60)

# Now call the actual function
result = kinematics.forward_kinematics(pulses, trims)
pos_function = np.array([result['x'], result['y'], result['z']])

print(f'Manual calculation:   X={pos_manual[0]:.2f}, Y={pos_manual[1]:.2f}, Z={pos_manual[2]:.2f}')
print(f'Function result:      X={pos_function[0]:.2f}, Y={pos_function[1]:.2f}, Z={pos_function[2]:.2f}')
print()

diff = pos_function - pos_manual
print(f'Difference:           dX={diff[0]:.4f}, dY={diff[1]:.4f}, dZ={diff[2]:.4f}')
print()

if np.allclose(pos_manual, pos_function, atol=0.01):
    print('✅ VERIFICATION PASSED: Manual and function results match!')
else:
    print('❌ VERIFICATION FAILED: Results do not match!')

print()
print('='*60)
