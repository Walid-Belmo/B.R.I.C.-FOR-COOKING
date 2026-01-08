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

def angle_to_pulse(angle, trim, joint_idx):
    """
    Converts a target angle (degrees) back to a servo pulse width.
    Inverse of pulse_to_angle.
    """
    # 1. Reverse Geometric Offset
    # angle = angle + offset  ->  angle_no_offset = angle - offset
    angle_no_offset = angle - cfg.JOINT_OFFSETS[joint_idx]
    
    # 2. Reverse Direction Inversion
    # angle = angle * direction  ->  angle_oriented = angle / direction
    # Since direction is +1 or -1, division is same as multiplication
    angle_oriented = angle_no_offset * cfg.JOINT_DIRECTIONS[joint_idx]
    
    # 3. Reverse Scale
    # angle = delta * scale  ->  delta = angle / scale
    delta = angle_oriented / cfg.DEG_PER_US[joint_idx]
    
    # 4. Add Neutral Pulse
    neutral_pulse = 1500 + trim
    pulse = neutral_pulse + delta
    
    return int(pulse)

# ==========================================
# SAFETY FUNCTIONS
# ==========================================

# Cylindrical base parameters (from incident report)
BASE_CYLINDER_RADIUS = 60.625 / 2  # mm (diameter 60.625mm)
BASE_CYLINDER_HEIGHT = 61.0  # mm

def check_collision_with_base(x, y, z):
    """
    Checks if a point collides with the cylindrical base.
    
    Args:
        x, y, z: Coordinates in mm
        
    Returns:
        bool: True if collision detected, False if safe
    """
    # Check if within cylinder height
    if z < 0 or z > BASE_CYLINDER_HEIGHT:
        return False  # Outside vertical range
    
    # Check if within cylinder radius
    distance_from_center = np.sqrt(x**2 + y**2)
    if distance_from_center < BASE_CYLINDER_RADIUS:
        return True  # COLLISION!
    
    return False  # Safe

def get_home_position_pulses():
    """
    Returns the safe home position: all joints at 0° (vertical).
    This is 1500us + trim for each joint.
    """
    home_pulses = []
    for i in range(4):
        home_pulses.append(1500 + cfg.DEFAULT_TRIMS[i])
    return home_pulses

def check_path_collision(start_pulses, end_pulses, num_samples=20):
    """
    Checks if a linear path in pulse space will cause collision with base.
    
    Args:
        start_pulses: Starting servo pulses [p1, p2, p3, p4]
        end_pulses: Ending servo pulses [p1, p2, p3, p4]
        num_samples: Number of points to check along the path
        
    Returns:
        dict: {'collision': bool, 'collision_point': (x, y, z) or None}
    """
    trims = cfg.DEFAULT_TRIMS
    
    for i in range(num_samples + 1):
        t = i / num_samples
        # Interpolate pulses linearly
        interpolated_pulses = [
            start_pulses[j] + t * (end_pulses[j] - start_pulses[j])
            for j in range(4)
        ]
        
        # Get position at this interpolated pose
        fk_result = forward_kinematics(interpolated_pulses, trims)
        x, y, z = fk_result['x'], fk_result['y'], fk_result['z']
        
        # Check collision
        if check_collision_with_base(x, y, z):
            return {
                'collision': True,
                'collision_point': (x, y, z),
                'progress': t
            }
    
    return {'collision': False, 'collision_point': None}

def inverse_kinematics_numerical(target_x, target_y, target_z, current_pulses=None, max_iter=50, tolerance=1.0):
    """
    Calculates the servo pulses required to reach (target_x, target_y, target_z).
    Uses a Numerical Jacobian approach (iterative).
    
    Args:
        target_x, target_y, target_z: Desired coordinates in mm.
        current_pulses: Starting guess [p1, p2, p3, p4]. If None, starts from Neutral.
        max_iter: Maximum iterations to try.
        tolerance: Distance error threshold in mm.
        
    Returns:
        dict: {
            'pulses': [p1, p2, p3, p4],
            'success': bool,
            'error': float,
            'iterations': int
        }
    """
    target_pos = np.array([target_x, target_y, target_z])
    
    # Start guess
    if current_pulses is None:
        pulses = [1500, 1500, 1500, 1500]
    else:
        pulses = list(current_pulses)
        
    trims = cfg.DEFAULT_TRIMS
    
    # Gradient Descent Parameters
    alpha = 0.5  # Learning rate (step size)
    
    for iteration in range(max_iter):
        # 1. Calculate Current Position with current pulses
        fk_result = forward_kinematics(pulses, trims)
        current_pos = np.array([fk_result['x'], fk_result['y'], fk_result['z']])
        
        # 2. Calculate Error Vector
        error_vec = target_pos - current_pos
        error_dist = np.linalg.norm(error_vec)
        
        if error_dist < tolerance:
            return {
                'pulses': [int(p) for p in pulses],
                'success': True,
                'error': error_dist,
                'iterations': iteration
            }
            
        # 3. Compute Numerical Jacobian (J)
        # J tells us: "How much does X/Y/Z change if I change Servo i by 1 unit?"
        J = np.zeros((3, 4))
        epsilon = 1.0 # Small perturbation in microseconds
        
        for i in range(4):
            # Perturb servo i
            perturbed_pulses = list(pulses)
            perturbed_pulses[i] += epsilon
            
            fk_perturbed = forward_kinematics(perturbed_pulses, trims)
            pos_perturbed = np.array([fk_perturbed['x'], fk_perturbed['y'], fk_perturbed['z']])
            
            # Derivative = (f(x+h) - f(x)) / h
            # Here, column i of J is the partial derivative vector
            col = (pos_perturbed - current_pos) / epsilon
            J[:, i] = col

        # 4. Solves for delta_pulses:  J * delta_pulses = error_vec
        # Using Pseudo-Inverse for stability
        # delta_pulses = pinv(J) * error_vec
        J_pinv = np.linalg.pinv(J)
        delta_pulses = J_pinv @ error_vec
        
        # 5. Update Pulses
        # pulses = pulses + alpha * delta_pulses
        for i in range(4):
            pulses[i] += alpha * delta_pulses[i]
            
            # Clamp to safety limits
            pulses[i] = max(500, min(2500, pulses[i]))
            
    # If we run out of iterations
    return {
        'pulses': [int(p) for p in pulses],
        'success': False,
        'error': error_dist,
        'iterations': max_iter
    }

def safe_move_to_target(target_x, target_y, target_z, current_pulses):
    """
    SAFETY-FIRST move function implementing the ESSENTIAL rule from incident report:
    1. Always return to vertical home position first (all angles 0°)
    2. Then move to the target position
    3. Check for collisions along both paths
    
    This prevents dangerous trajectories and ensures predictable movement.
    
    Args:
        target_x, target_y, target_z: Target coordinates in mm
        current_pulses: Current servo positions [p1, p2, p3, p4]
        
    Returns:
        dict: {
            'success': bool,
            'error': str or None,
            'path': list of pulse positions to execute,
            'collision_check': dict
        }
    """
    # 1. First check if target itself is in collision
    if check_collision_with_base(target_x, target_y, target_z):
        return {
            'success': False,
            'error': f'Target ({target_x:.1f}, {target_y:.1f}, {target_z:.1f}) collides with base!',
            'path': None,
            'collision_check': {'target_collision': True}
        }
    
    # 2. Get home position
    home_pulses = get_home_position_pulses()
    
    # 3. Check path from current position to home
    collision_to_home = check_path_collision(current_pulses, home_pulses)
    if collision_to_home['collision']:
        return {
            'success': False,
            'error': f'Path to home position collides at {collision_to_home["collision_point"]}',
            'path': None,
            'collision_check': collision_to_home
        }
    
    # 4. Calculate IK for target position (starting from home)
    ik_result = inverse_kinematics_numerical(target_x, target_y, target_z, 
                                            current_pulses=home_pulses, 
                                            max_iter=50, 
                                            tolerance=1.0)
    
    if not ik_result['success']:
        return {
            'success': False,
            'error': f'IK failed: error={ik_result["error"]:.2f}mm, iterations={ik_result["iterations"]}',
            'path': None,
            'collision_check': {'ik_failed': True}
        }
    
    target_pulses = ik_result['pulses']
    
    # 5. Check path from home to target
    collision_to_target = check_path_collision(home_pulses, target_pulses)
    if collision_to_target['collision']:
        return {
            'success': False,
            'error': f'Path from home to target collides at {collision_to_target["collision_point"]}',
            'path': None,
            'collision_check': collision_to_target
        }
    
    # 6. All checks passed - return the safe two-step path
    return {
        'success': True,
        'error': None,
        'path': [home_pulses, target_pulses],
        'collision_check': {
            'to_home': collision_to_home,
            'to_target': collision_to_target
        },
        'ik_result': ik_result
    }

