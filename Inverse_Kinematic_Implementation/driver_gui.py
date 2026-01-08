import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import time
import threading
import datetime
import os
import kinematics

# ==========================================
# CONFIGURATION
# ==========================================
DEFAULT_SERVO_PINS = [13, 14, 22, 23]
BAUD_RATE = 115200
LOG_DIR = "logs"

# Servo Calibration
PULSE_MIN = 500
PULSE_MAX = 2500
NEUTRAL = 1500

class RobotArmApp:
    def __init__(self, root):
        self.root = root
        self.root.title("4-DOF Robot Arm Control & Kinematics")
        self.root.geometry("800x800")

        self.serial_port = None
        self.is_connected = False
        
        self.servo_values = [tk.IntVar(value=1500) for _ in range(4)]
        self.last_sent_values = [1500] * 4

        # Trims - imported from kinematics_config
        self.trims = list(kinematics.cfg.DEFAULT_TRIMS) 
        
        # IK Target Variables
        self.target_x = tk.DoubleVar(value=0.0)
        self.target_y = tk.DoubleVar(value=0.0)
        self.target_z = tk.DoubleVar(value=150.0)

        # Jog Variables
        self.jog_step_size = tk.DoubleVar(value=20.0)
        self.jog_speed = tk.IntVar(value=5) # 1 to 10
        self.is_moving = False

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
        with open(self.log_file_path, "a", encoding="utf-8") as f:
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

        # --- Inverse Kinematics Control Frame ---
        ik_frame = ttk.LabelFrame(self.root, text="Cartesian Control (Inverse Kinematics)")
        ik_frame.pack(fill="x", padx=10, pady=5)
        
        ik_input_frame = ttk.Frame(ik_frame)
        ik_input_frame.pack(pady=5)
        
        ttk.Label(ik_input_frame, text="X (mm):").pack(side="left", padx=2)
        ttk.Entry(ik_input_frame, textvariable=self.target_x, width=8).pack(side="left", padx=5)
        
        ttk.Label(ik_input_frame, text="Y (mm):").pack(side="left", padx=2)
        ttk.Entry(ik_input_frame, textvariable=self.target_y, width=8).pack(side="left", padx=5)
        
        ttk.Label(ik_input_frame, text="Z (mm):").pack(side="left", padx=2)
        ttk.Entry(ik_input_frame, textvariable=self.target_z, width=8).pack(side="left", padx=5)
        
        ttk.Button(ik_input_frame, text="Move To Target", command=self.move_to_coordinate).pack(side="left", padx=15)
        self.lbl_ik_status = ttk.Label(ik_frame, text="", foreground="blue")
        self.lbl_ik_status.pack(pady=2)

        # --- Jog Control Frame ---
        jog_frame = ttk.LabelFrame(self.root, text="Manual Jogging (XYZ)")
        jog_frame.pack(fill="x", padx=10, pady=5)
        
        # Settings (Step Size & Speed)
        jog_settings_frame = ttk.Frame(jog_frame)
        jog_settings_frame.pack(fill="x", padx=5, pady=2)
        
        ttk.Label(jog_settings_frame, text="Step (mm):").pack(side="left")
        ttk.Entry(jog_settings_frame, textvariable=self.jog_step_size, width=5).pack(side="left", padx=5)
        
        ttk.Label(jog_settings_frame, text="Speed (1-10):").pack(side="left", padx=10)
        ttk.Scale(jog_settings_frame, from_=1, to=10, variable=self.jog_speed, orient="horizontal", length=100).pack(side="left")
        ttk.Label(jog_settings_frame, textvariable=self.jog_speed, width=3).pack(side="left")
        
        # Directional Buttons
        btn_frame = ttk.Frame(jog_frame)
        btn_frame.pack(pady=5)
        
        # Grid layout for buttons
        #       [Z+]
        # [X-]  [Y+]  [X+]
        #       [Y-]
        #       [Z-]
        
        # Middle Cluster (X/Y)
        ttk.Button(btn_frame, text="Y+ (Left)", width=10, command=lambda: self.perform_jog(0, 1, 0)).grid(row=1, column=1, padx=2, pady=2)
        ttk.Button(btn_frame, text="Y- (Right)", width=10, command=lambda: self.perform_jog(0, -1, 0)).grid(row=3, column=1, padx=2, pady=2)
        
        ttk.Button(btn_frame, text="X+ (Fwd)", width=10, command=lambda: self.perform_jog(1, 0, 0)).grid(row=2, column=2, padx=2, pady=2)
        ttk.Button(btn_frame, text="X- (Back)", width=10, command=lambda: self.perform_jog(-1, 0, 0)).grid(row=2, column=0, padx=2, pady=2)
        
        # Vertical (Z)
        ttk.Button(btn_frame, text="Z+ (Up)", width=10, command=lambda: self.perform_jog(0, 0, 1)).grid(row=0, column=1, padx=2, pady=10)
        ttk.Button(btn_frame, text="Z- (Down)", width=10, command=lambda: self.perform_jog(0, 0, -1)).grid(row=4, column=1, padx=2, pady=10)

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


    def update_kinematics_loop(self):
        # 1. Read current slider targets
        current_pulses = [v.get() for v in self.servo_values]

        # 2. Send commands if changed significantly (simple optimization)
        for i in range(4):
            if abs(current_pulses[i] - self.last_sent_values[i]) >= 5: # 5us deadband
                self.send_command(i, current_pulses[i])
                self.last_sent_values[i] = current_pulses[i]
                self.log_message(f"Servo {i+1} set to {current_pulses[i]}")

        # 3. Calculate Kinematics using new modular system
        result = kinematics.forward_kinematics(current_pulses, self.trims)

        # Extract results
        angles = result['angles']
        x, y, z = result['x'], result['y'], result['z']

        # Update displays
        self.lbl_angles.config(text=f"Angles (deg): q1={angles[0]:.1f}, q2={angles[1]:.1f}, q3={angles[2]:.1f}, q4={angles[3]:.1f}")
        self.lbl_pos.config(text=f"X: {x:.2f} mm | Y: {y:.2f} mm | Z: {z:.2f} mm")

        self.root.after(100, self.update_kinematics_loop) # 10Hz update

    def perform_jog(self, dx_dir, dy_dir, dz_dir):
        """
        Calculates and executes a relative move (jog).
        """
        if self.is_moving:
            return # Ignore clicks while moving

        try:
            step = self.jog_step_size.get()
        except ValueError:
            return

        dx = dx_dir * step
        dy = dy_dir * step
        dz = dz_dir * step
        
        current_pulses = [v.get() for v in self.servo_values]
        
        # 1. Check Safety
        result = kinematics.compute_safe_jog(dx, dy, dz, current_pulses)
        
        if not result['success']:
            self.lbl_ik_status.config(text=f"JOG BLOCKED: {result['error']}", foreground="red")
            self.log_message(f"Jog Blocked: {result['error']}")
            return
            
        # 2. Execute Move
        target_pulses = result['pulses']
        speed = self.jog_speed.get()
        
        self.log_message(f"Jogging ({dx:.0f}, {dy:.0f}, {dz:.0f}) Speed={speed}...")
        self.execute_smooth_move(current_pulses, target_pulses, speed)

    def execute_smooth_move(self, start_pulses, target_pulses, speed):
        """
        Interpolates from start to target pulses based on speed setting.
        Speed 10 = Jump (Immediate)
        Speed 1 = Slowest
        """
        self.is_moving = True
        
        if speed >= 10:
            # Immediate Jump
            for i in range(4):
                self.servo_values[i].set(target_pulses[i])
            self.is_moving = False
            self.lbl_ik_status.config(text="Jog Complete", foreground="green")
            return

        # Calculate animation frames
        # speed 1 -> 50 steps
        # speed 9 -> 5 steps
        steps = int(50 / speed) 
        if steps < 5: steps = 5 # Minimum smoothness
        
        # Determine pulse deltas
        deltas = [(target_pulses[i] - start_pulses[i]) / steps for i in range(4)]
        
        def animate(step_idx):
            if step_idx >= steps:
                # Finalize to exact target
                for i in range(4):
                    self.servo_values[i].set(target_pulses[i])
                self.is_moving = False
                self.lbl_ik_status.config(text="Jog Complete", foreground="green")
                return
            
            # Apply intermediate values
            for i in range(4):
                val = start_pulses[i] + deltas[i] * (step_idx + 1)
                self.servo_values[i].set(int(val))
            
            # Schedule next frame (20ms = 50Hz)
            self.root.after(20, lambda: animate(step_idx + 1))
            
        animate(0)

    def move_to_coordinate(self):
        try:
            tx = self.target_x.get()
            ty = self.target_y.get()
            tz = self.target_z.get()
        except ValueError:
            self.lbl_ik_status.config(text="Invalid Coordinates", foreground="red")
            return

        self.log_message(f"=== SAFE MOVE REQUEST: Target ({tx}, {ty}, {tz}) ===")
        
        # Get current pulses
        current_pulses = [v.get() for v in self.servo_values]
        
        # Use SAFE move function (goes to home first)
        result = kinematics.safe_move_to_target(tx, ty, tz, current_pulses)
        
        if result['success']:
            # Extract the two-step path: [home_pulses, target_pulses]
            home_pulses, target_pulses = result['path']
            ik_result = result['ik_result']
            
            self.log_message(f"✓ Path validated - No collisions detected")
            self.log_message(f"✓ IK converged: {ik_result['iterations']} iterations, error: {ik_result['error']:.2f}mm")
            self.log_message(f"Step 1: Moving to HOME position (vertical)...")
            
            # Move to home first
            for i in range(4):
                self.servo_values[i].set(home_pulses[i])
            
            # Give time for servos to reach home (you might want to add a delay or confirmation)
            self.root.after(2000, lambda: self.execute_target_move(target_pulses, ik_result))
            
            self.lbl_ik_status.config(text=f"Moving via HOME (Err: {ik_result['error']:.2f}mm)", foreground="blue")
        else:
            # Failed - show error
            self.log_message(f"✗ MOVE ABORTED: {result['error']}")
            self.lbl_ik_status.config(text=f"BLOCKED: {result['error'][:40]}...", foreground="red")
    
    def execute_target_move(self, target_pulses, ik_result):
        """Called after home position is reached"""
        self.log_message(f"Step 2: Moving to TARGET position...")
        
        # Update sliders to target
        for i in range(4):
            self.servo_values[i].set(target_pulses[i])
        
        self.log_message(f"✓ Movement complete!")
        self.lbl_ik_status.config(text=f"Success (Err: {ik_result['error']:.2f}mm)", foreground="green")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmApp(root)
    root.mainloop()
