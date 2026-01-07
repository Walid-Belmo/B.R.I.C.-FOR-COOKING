import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import kinematics_config as cfg

# ==========================================
# GEOMETRY CONFIGURATION
# ==========================================
# All units in mm, imported from kinematics_config.py

# 1. Base (Origin) to Shoulder (J2)
J1_TO_J2_Z = cfg.J1_TO_J2_Z
J1_TO_J2_Y = cfg.J1_TO_J2_Y
J1_TO_J2_X = cfg.J1_TO_J2_X

# 2. Shoulder (J2) to Elbow (J3)
J2_TO_J3_LEN = cfg.J2_TO_J3_LEN

# 3. Elbow (J3) to Wrist (J4)
J3_TO_J4_X = cfg.J3_TO_J4_X
J3_TO_J4_Y = cfg.J3_TO_J4_Y
J3_TO_J4_Z = cfg.J3_TO_J4_Z

# 4. Wrist (J4) to Tip
J4_TO_TIP_X = cfg.J4_TO_TIP_X
J4_TO_TIP_Y = cfg.J4_TO_TIP_Y
J4_TO_TIP_Z = cfg.J4_TO_TIP_Z

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

def get_translation_matrix(x, y, z):
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])

def calculate_chain(q1, q2, q3, q4):
    # Points to plot: Origin, J2_loc, J3_loc, J4_loc, Tip_loc
    points = []
    
    # 0. World Origin
    T_world = np.eye(4)
    points.append(T_world[:3, 3])
    
    # --- TRANSFORMATION CHAIN ---
    
    # 1. Base Rotation (J1) - Axis Z
    R_j1 = get_rotation_matrix('z', q1)
    
    # 2. Move to J2 (Shoulder)
    # The shift happens AFTER J1 rotation (it orbits)? 
    # Or is the servo mounted ON the shift?
    # User said: "base servo has a turntable... that has servo 2 mounted... shifted in Y"
    # This implies the shift rotates WITH J1.
    T_j1_j2 = get_translation_matrix(J1_TO_J2_X, J1_TO_J2_Y, J1_TO_J2_Z)
    
    # Current Matrix: World -> Rotate J1 -> Translate to J2
    M_j2 = T_world @ R_j1 @ T_j1_j2
    points.append(M_j2[:3, 3])
    
    # 3. Shoulder Rotation (J2) - Axis X
    R_j2 = get_rotation_matrix('x', q2)
    
    # 4. Move to J3 (Elbow)
    # "Shifted only vertically" (along the link)
    T_j2_j3 = get_translation_matrix(0, 0, J2_TO_J3_LEN)
    
    M_j3 = M_j2 @ R_j2 @ T_j2_j3
    points.append(M_j3[:3, 3])
    
    # 5. Elbow Rotation (J3) - Axis X
    R_j3 = get_rotation_matrix('x', q3)
    
    # 6. Move to J4 (Wrist)
    # "Move along Z, Y and X"
    T_j3_j4 = get_translation_matrix(J3_TO_J4_X, J3_TO_J4_Y, J3_TO_J4_Z)
    
    M_j4 = M_j3 @ R_j3 @ T_j3_j4
    points.append(M_j4[:3, 3])
    
    # 7. Wrist Rotation (J4) - Axis Z (Yaw)
    # User said: "servo 1 and 4 have rotational axis around Z"
    R_j4 = get_rotation_matrix('z', q4)
    
    # 8. Move to Tip
    T_j4_tip = get_translation_matrix(J4_TO_TIP_X, J4_TO_TIP_Y, J4_TO_TIP_Z)
    
    M_tip = M_j4 @ R_j4 @ T_j4_tip
    points.append(M_tip[:3, 3])
    
    return np.array(points)

# ==========================================
# PLOTTING
# ==========================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.25)

# Sliders
ax_q1 = plt.axes([0.25, 0.1, 0.65, 0.03])
ax_q2 = plt.axes([0.25, 0.07, 0.65, 0.03])
ax_q3 = plt.axes([0.25, 0.04, 0.65, 0.03])
ax_q4 = plt.axes([0.25, 0.01, 0.65, 0.03])

s_q1 = Slider(ax_q1, 'J1 (Base Z)', -180, 180, valinit=0)
s_q2 = Slider(ax_q2, 'J2 (Shldr X)', -180, 180, valinit=0)
s_q3 = Slider(ax_q3, 'J3 (Elbow X)', -180, 180, valinit=0)
s_q4 = Slider(ax_q4, 'J4 (Wrist Z)', -180, 180, valinit=0)

def update(val):
    ax.clear()
    
    # Fixed Plot Limits for stability
    LIMIT = 300
    ax.set_xlim(-LIMIT, LIMIT)
    ax.set_ylim(-LIMIT, LIMIT)
    ax.set_zlim(0, 400)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # Calculate Chain
    q1, q2, q3, q4 = s_q1.val, s_q2.val, s_q3.val, s_q4.val
    pts = calculate_chain(q1, q2, q3, q4)
    
    # Plot Bones (Blue Lines)
    ax.plot(pts[:,0], pts[:,1], pts[:,2], '-o', linewidth=4, markersize=8, color='blue')
    
    # Plot Joints (Red dots)
    ax.scatter(pts[:,0], pts[:,1], pts[:,2], color='red')
    
    # Text Annotations
    labels = ['Base', 'J2', 'J3', 'J4', 'Tip']
    for i, txt in enumerate(labels):
        ax.text(pts[i,0], pts[i,1], pts[i,2], txt, fontsize=10)

    # Orientation (Draw local axes at Tip)
    # (Simplified for visual clarity - just showing position mostly)
    
    fig.canvas.draw_idle()

s_q1.on_changed(update)
s_q2.on_changed(update)
s_q3.on_changed(update)
s_q4.on_changed(update)

update(0)
plt.show()
