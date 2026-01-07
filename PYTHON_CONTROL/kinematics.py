import numpy as np
import kinematics_config as cfg

def get_rotation_matrix(axis, angle_deg):
    rad = np.radians(angle_deg)
    c = np.cos(rad)
    s = np.sin(rad)
    
    if axis == 'x':
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
    elif axis == 'y':
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1]
        ])
    elif axis == 'z':
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    return np.eye(4)

def get_translation_matrix(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def pulse_to_angle(pulse, trim, joint_idx):
    # Calculates angle in degrees from pulse width.
    # We assume '1500 + trim' is the Neutral (0 degree) position.
    neutral_pulse = 1500 + trim
    delta = pulse - neutral_pulse
    
    # 1. Scale to Degrees
    # Use list of scales for different servo models (e.g. J4 is MG90S)
    angle = delta * cfg.DEG_PER_US[joint_idx]
    
    # 2. Apply Direction Inversion
    angle = angle * cfg.JOINT_DIRECTIONS[joint_idx]
    
    # 3. Apply Geometric Offset (e.g. if 1500us is at 45 degrees physically)
    angle = angle + cfg.JOINT_OFFSETS[joint_idx]
    
    return angle

def forward_kinematics(pulses, trims=None):
    """
    Calculates the Tip Position (X, Y, Z) based on servo pulses.
    
    Args:
        pulses: list of 4 integers [p1, p2, p3, p4]
        trims: list of 4 integers [t1, t2, t3, t4]. If None, uses defaults.
    
    Returns:
        dict: {'x': float, 'y': float, 'z': float, 'angles': [q1, q2, q3, q4]}
    """
    if trims is None:
        trims = cfg.DEFAULT_TRIMS

    # 1. Convert Pulses to Angles
    angles = []
    for i in range(4):
        angles.append(pulse_to_angle(pulses[i], trims[i], i))
    
    q1, q2, q3, q4 = angles

    # 2. Build Transformation Chain
    
    # --- Frame 0 (Base) -> Frame 1 (Turntable Rotation) ---
    # J1 rotates around Z
    R_j1 = get_rotation_matrix('z', q1)
    M1 = R_j1 
    
    # --- Frame 1 -> Frame 2 (Shoulder Pivot) ---
    # Translation from Base Center to Shoulder Center
    # Then J2 rotates around X (Pitch)
    T_1_2 = get_translation_matrix(cfg.J1_TO_J2_X, cfg.J1_TO_J2_Y, cfg.J1_TO_J2_Z)
    R_j2 = get_rotation_matrix('x', q2)
    M2 = M1 @ T_1_2 @ R_j2

    # --- Frame 2 -> Frame 3 (Elbow Pivot) ---
    # Translation along the Humerus (Vertical Z in local frame)
    # Then J3 rotates around X (Pitch)
    T_2_3 = get_translation_matrix(0, 0, cfg.J2_TO_J3_LEN)
    R_j3 = get_rotation_matrix('x', q3)
    M3 = M2 @ T_2_3 @ R_j3

    # --- Frame 3 -> Frame 4 (Wrist Pivot) ---
    # Translation along Forearm (with complex XYZ offsets)
    # Then J4 rotates around Z (Yaw/Roll) !!! Important difference from J2/J3
    T_3_4 = get_translation_matrix(cfg.J3_TO_J4_X, cfg.J3_TO_J4_Y, cfg.J3_TO_J4_Z)
    R_j4 = get_rotation_matrix('z', q4)
    M4 = M3 @ T_3_4 @ R_j4

    # --- Frame 4 -> Tool Tip ---
    # Translation to the end effector
    T_4_Tip = get_translation_matrix(cfg.J4_TO_TIP_X, cfg.J4_TO_TIP_Y, cfg.J4_TO_TIP_Z)
    M_Tip = M4 @ T_4_Tip

    # Extract Position
    tip_pos = M_Tip[:3, 3] # [x, y, z]

    return {
        'x': tip_pos[0],
        'y': tip_pos[1],
        'z': tip_pos[2],
        'angles': angles
    }
