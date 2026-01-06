import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import math
import time
import threading
import datetime
import os

# ==========================================
# CONFIGURATION
# ==========================================
DEFAULT_SERVO_PINS = [13, 14, 22, 23]
BAUD_RATE = 115200
LOG_DIR = "logs"

# Robot Geometry Constants (mm) derived from V4 code
# Adjust these to match physical measurements exactly
H_BASE = 62.3
J2_OFF = [0.0, 16.625, 36.276]  # [x, y, z] from Base logical origin to J2 axis
J3_OFF = [0.0, 0.0, 120.0]      # [x, y, z] from J2 to J3
J4_OFF = [11.08, 0.0, 93.85]    # [x, y, z] from J3 to J4
TCP_OFF = [0.0, 4.9, 45.6]      # [x, y, z] from J4 to Tool Center Point

# Servo Calibration
# Pulse width (us) matching V4
PULSE_MIN = 500
PULSE_MAX = 2500
NEUTRAL = 1500

# Conversion factors (approximate from V4)
# Scale 270 deg = 1.5 PI / 2000 pulses?
# 2000 pulses range (2500-500). 
# Range angle = 270 degrees.
DEG_PER_US_270 = 270.0 / 2000.0
DEG_PER_US_180 = 180.0 / 2000.0

class RobotArmApp:
    def __init__(self, root):
        self.root = root
        self.root.title("4-DOF Robot Arm Control & Kinematics")
        self.root.geometry("800x800")

        self.serial_port = None
        self.is_connected = False
        
        self.servo_values = [tk.IntVar(value=1500) for _ in range(4)]
        self.last_sent_values = [1500] * 4
        
        # Trims (to calibrate 0 degrees to 1500us or other)
        # Based on V4 code: {0, -50, 0, 0}
        self.trims = [0, -50, 0, 0] 

        self.setup_ui()
        self.setup_logging()
        
        # Start kinematics update loop
        self.update_kinematics_loop()

    def setup_logging(self):
        if not os.path.exists(LOG_DIR):
            os.makedirs(LOG_DIR)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = os.path.join(LOG_DIR, f"robot_log_{timestamp}.txt")
        self.log_message(f"Session started: {timestamp}")

    def log_message(self, msg):
        timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        full_msg = f"[{timestamp}] {msg}"
        
        # UI Log
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, full_msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')
        
        # File Log
        with open(self.log_file_path, "a") as f:
            f.write(full_msg + "\n")

    def setup_ui(self):
        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        self.port_combo = ttk.Combobox(conn_frame, values=self.get_serial_ports())
        self.port_combo.pack(side="left", padx=5, pady=5)
        
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5)
        
        self.btn_refresh = ttk.Button(conn_frame, text="Refresh Ports", command=lambda: self.port_combo.config(values=self.get_serial_ports()))
        self.btn_refresh.pack(side="left", padx=5)

        # --- Control Frame ---
        control_frame = ttk.LabelFrame(self.root, text="Servo Control (Pulse Width us)")
        control_frame.pack(fill="x", padx=10, pady=5)

        for i in range(4):
            frame = ttk.Frame(control_frame)
            frame.pack(fill="x", padx=5, pady=5)
            
            lbl = ttk.Label(frame, text=f"Servo {i+1} (J{i+1}):", width=15)
            lbl.pack(side="left")
            
            # Slider
            scale = ttk.Scale(frame, from_=500, to=2500, variable=self.servo_values[i], orient="horizontal", command=lambda v, idx=i: self.on_slider_change(idx, v))
            scale.pack(side="left", fill="x", expand=True, padx=5)
            
            # Value Label
            val_lbl = ttk.Label(frame, textvariable=self.servo_values[i], width=6)
            val_lbl.pack(side="left")

        # --- Kinematics Frame ---
        kin_frame = ttk.LabelFrame(self.root, text="Forward Kinematics Estimation (Calculated)")
        kin_frame.pack(fill="x", padx=10, pady=5)
        
        self.lbl_pos = ttk.Label(kin_frame, text="X: 0.00 mm | Y: 0.00 mm | Z: 0.00 mm", font=("Consolas", 14, "bold"))
        self.lbl_pos.pack(pady=10)
        
        self.lbl_angles = ttk.Label(kin_frame, text="Angles (deg): q1=0, q2=0, q3=0, q4=0")
        self.lbl_angles.pack(pady=5)

        # --- Log Frame ---
        log_frame = ttk.LabelFrame(self.root, text="Logs")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, state='disabled', height=10)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)

    def get_serial_ports(self):
        return [comport.device for comport in serial.tools.list_ports.comports()]

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combo.get()
            if not port:
                self.log_message("Error: No port selected")
                return
            try:
                self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.1)
                self.is_connected = True
                self.btn_connect.config(text="Disconnect")
                self.log_message(f"Connected to {port}")
            except Exception as e:
                self.log_message(f"Connection Failed: {e}")
        else:
            if self.serial_port:
                self.serial_port.close()
            self.is_connected = False
            self.btn_connect.config(text="Connect")
            self.log_message("Disconnected")

    def on_slider_change(self, index, value):
        # Debounce or limit sending rate could be added here
        pass

    def send_command(self, servo_idx, pulse_width):
        if self.is_connected and self.serial_port:
            cmd = f"s{servo_idx+1}-{int(pulse_width)}\n"
            try:
                self.serial_port.write(cmd.encode())
                # self.log_message(f"TX: {cmd.strip()}") 
                # Uncomment above for verbose TX logging, but it might spam
            except Exception as e:
                self.log_message(f"TX Error: {e}")

    def get_angle_deg(self, index, pulse):
        # Logic to convert Pulse (500-2500) to Angle (Degrees)
        # Based on V4 Scale.
        # Check if J3 is 180 or 270 (V4 says index 3 (J4) is 180-scale, others 270?)
        # Wait, V4 code: "if (index == 3) angle = deltaPulse * SCALE_180; else ... SCALE_270"
        # Index 3 in C++ is J4 (0,1,2,3).
        
        center = NEUTRAL + self.trims[index]
        delta = pulse - center
        
        if index == 3: # J4
            angle = delta * DEG_PER_US_180
        else:
            angle = delta * DEG_PER_US_270
            
        # V4 Inversion: if (index == 2) return -angle; // J3 Inverted in V4? (index 2 is J3)
        if index == 2:
            return -angle
        return angle

    def update_kinematics_loop(self):
        # 1. Read current slider targets
        current_pulses = [v.get() for v in self.servo_values]
        
        # 2. Send commands if changed significantly (simple optimization)
        for i in range(4):
            if abs(current_pulses[i] - self.last_sent_values[i]) >= 5: # 5us deadband
                self.send_command(i, current_pulses[i])
                self.last_sent_values[i] = current_pulses[i]
                self.log_message(f"Servo {i+1} set to {current_pulses[i]}")

        # 3. Calculate Kinematics
        angles = [self.get_angle_deg(i, current_pulses[i]) for i in range(4)]
        q_rad = [math.radians(a) for a in angles]
        
        # Update Angle Display
        self.lbl_angles.config(text=f"Angles (deg): q1={angles[0]:.1f}, q2={angles[1]:.1f}, q3={angles[2]:.1f}, q4={angles[3]:.1f}")

        # --- GEOMETRIC CALCULATION ---
        # Base frame: Z up, X forward, Y left (standard right hand rule)
        
        # J1 Rotation (Yaw around Z)
        q1 = q_rad[0]
        c1, s1 = math.cos(q1), math.sin(q1)
        
        # Position of Shoulder Pivot (J2) relative to Base Origin
        # J2_OFF contains standard offsets.
        # Rotated by J1.
        # Assuming J2_OFF[0]=x, J2_OFF[1]=y, J2_OFF[2]=z
        
        # Vector from Base to J2 (applying J1 rotation)
        x2 = J2_OFF[0] * c1 - J2_OFF[1] * s1
        y2 = J2_OFF[0] * s1 + J2_OFF[1] * c1
        z2 = H_BASE + J2_OFF[2]
        
        # J2 Rotation (Pitch around local Y')
        q2 = q_rad[1]
        
        # J3 moves relative to J2. It's a pitch joint.
        # Pitch plane logic:
        # Radial distance r = link_len * sin(angle) ?? No, normally r = L * cos(pitch)
        # Vertical z = L * sin(pitch)
        # Let's verify "0" position. 
        # If Q2=0 is UP (Vertical):
        #   x_local = J3_OFF[2] * sin(q2)
        #   z_local = J3_OFF[2] * cos(q2)
        # If Q2=0 is Forward (Horizontal):
        #   x_local = J3_OFF[2] * cos(q2)
        #   z_local = J3_OFF[2] * sin(q2)
        
        # Looking at V4 code: j3_z = j2_z + (J3_OFF[2] * c2);
        # This implies when q2=0, z is max. So 0 is VERTICAL UP.
        
        # Vector J2->J3 (Length J3_OFF[2] = 120)
        # Rotated by q2 (Pitch).
        # Projects onto XY plane (radial ground distance) usually as L*sin(q2)
        r_j2_j3 = J3_OFF[2] * math.sin(q2)
        dz_j2_j3 = J3_OFF[2] * math.cos(q2)
        
        # Add J3_OFF Y offset if exists (it is 0.0 in V4)
        # But J3_OFF[2] is the Z-length component.
        
        # J3 Position
        x3 = x2 + r_j2_j3 * c1
        y3 = y2 + r_j2_j3 * s1
        z3 = z2 + dz_j2_j3
        
        # J3 Rotation (Pitch)
        q3 = q_rad[2]
        # Total Pitch for next link
        pitch_sum_j3 = q2 + q3
        
        # Vector J3->J4
        # J4_OFF[0] is X offset (thickness?), J4_OFF[2] is Z/Length (93.85)
        # Assuming similar convention: 0 is Straight w.r.t previous link?
        # V4 code: j4_z = j3_z + (J4_OFF[2] * c23); -> c23 = cos(q2+q3)
        # This confirms 0 is aligned with previous vertical, or accumulated angle from vertical.
        
        r_j3_j4 = J4_OFF[2] * math.sin(pitch_sum_j3)
        dz_j3_j4 = J4_OFF[2] * math.cos(pitch_sum_j3)
        
        # Also handle J4_OFF[0] (11.08). Is it perpendicular?
        # V4: j4_x = j3_x + (J4_OFF[0] * c1) ... 
        # It adds J4_OFF[0] directly along X/Y of base J1. 
        # This means J4_OFF[0] does NOT rotate with Pitch? That's weird.
        # It implies an offset along the J3 axis of rotation? (Side offset)
        # If J4_OFF[0] is multiplied by c1/s1, it means it's a radial offset in the direction of the arm's azimuth.
        # That sounds like it's adding to the radius `r`.
        
        r_j3_j4_total = r_j3_j4 + J4_OFF[0]
        
        x4 = x3 + r_j3_j4_total * c1
        y4 = y3 + r_j3_j4_total * s1
        z4 = z3 + dz_j3_j4
        
        # J4 Rotation (User says "not a gripper").
        # If it's a 4th DOF Pitch link:
        q4 = q_rad[3]
        pitch_sum_j4 = pitch_sum_j3 + q4
        
        # Vector J4->TCP
        # TCP_OFF[2] (45.6) is length.
        # V4 code: targetZ = j4_z + (TCP_OFF[2] * c23); 
        # V4 IGNORED q4 for pitch. We will INCLUDE it.
        # If J4 is indeed the 4th servo pitch:
        
        r_j4_tcp = TCP_OFF[2] * math.sin(pitch_sum_j4)
        dz_j4_tcp = TCP_OFF[2] * math.cos(pitch_sum_j4)
        
        # Add TCP offsets
        # V4: TCP_OFF[1] * s1 ...
        # If TCP_OFF[1] is Y offset defined in Tool Frame?
        # If tool has offset 4.9mm
        
        # Refined Model for X, Y, Z final:
        xf = x4 + r_j4_tcp * c1
        yf = y4 + r_j4_tcp * s1
        zf = z4 + dz_j4_tcp
        
        # Update Labels
        self.lbl_pos.config(text=f"X: {xf:.2f} | Y: {yf:.2f} | Z: {zf:.2f}")

        # Store calculated pos for logging?
        # Maybe log only periodically or on request to avoid IO lag
        
        self.root.after(100, self.update_kinematics_loop) # 10Hz update

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmApp(root)
    root.mainloop()
