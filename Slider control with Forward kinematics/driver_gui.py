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
DEFAULT_SERVO_PINS = [13, 14, 22, 23, 21]
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
        
        self.servo_values = [tk.IntVar(value=1500) for _ in range(5)]
        self.last_sent_values = [1500] * 5

        # Trims - imported from kinematics_config
        self.trims = list(kinematics.cfg.DEFAULT_TRIMS) 

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

        ttk.Button(control_frame, text="Set All to 1500", command=self.set_all_1500).pack(pady=5)

        for i in range(5):
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

    def set_all_1500(self):
        for i, var in enumerate(self.servo_values):
            var.set(1500)
            self.last_sent_values[i] = -1 # Force update in loop
        self.log_message("All servos set to 1500us (Raw)")

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
        for i in range(5):
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

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmApp(root)
    root.mainloop()
