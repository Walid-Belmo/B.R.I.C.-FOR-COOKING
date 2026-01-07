import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import kinematics_config as cfg
import os

# ==========================================
# INTERACTIVE CALIBRATION TOOL
# ==========================================
# Goal: Find the correct Direction and Offset for each servo
# so that Pulse 1500 +/- Offset = 0 Degrees (Vertical).

class CalibrationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Arm Calibrator (Debug Tool)")
        self.root.geometry("800x600")
        
        self.serial_port = None
        self.is_connected = False
        
        # Load existing config defaults
        self.directions = list(cfg.JOINT_DIRECTIONS)
        self.offsets = list(cfg.JOINT_OFFSETS)
        
        # UI Variables
        self.pulse_vars = [tk.IntVar(value=1500) for _ in range(4)]
        self.dir_vars = [tk.IntVar(value=d) for d in self.directions] # 1 or -1
        self.offset_vars = [tk.DoubleVar(value=o) for o in self.offsets]
        
        self.calc_angle_labels = []

        self.setup_ui()
        
    def setup_ui(self):
        # --- HEADER ---
        frame_top = ttk.Frame(self.root)
        frame_top.pack(fill="x", padx=10, pady=10)
        
        self.port_combo = ttk.Combobox(frame_top, values=self.get_serial_ports())
        self.port_combo.pack(side="left")
        
        btn_connect = ttk.Button(frame_top, text="Connect", command=self.toggle_connection)
        btn_connect.pack(side="left", padx=10)
        
        btn_refresh = ttk.Button(frame_top, text="Refresh", command=lambda: self.port_combo.config(values=self.get_serial_ports()))
        btn_refresh.pack(side="left")
        
        btn_save = ttk.Button(frame_top, text="SAVE CONFIG", command=self.save_config, style="Accent.TButton")
        btn_save.pack(side="right", padx=10)

        # --- INSTRUCTIONS ---
        lbl_instr = ttk.Label(self.root, text="INSTRUCTIONS:\n1. Move Slider until physical link is VERTICAL (Pointed at ceiling).\n2. Adjust OFFSET until 'Calc Angle' reads 0.0.\n3. Move Slider Right. If arm goes DOWN, set Direction to -1.", foreground="blue", justify="left")
        lbl_instr.pack(pady=10)

        # --- JOINTS ---
        joints_frame = ttk.Frame(self.root)
        joints_frame.pack(fill="both", expand=True, padx=20)
        
        for i in range(4):
            # Frame for each joint
            row = ttk.LabelFrame(joints_frame, text=f"Joint J{i+1}")
            row.pack(fill="x", pady=5)
            
            # Col 1: Slider (Pulse)
            f_slider = ttk.Frame(row)
            f_slider.pack(side="left", fill="x", expand=True, padx=5)
            
            scale = ttk.Scale(f_slider, from_=500, to=2500, variable=self.pulse_vars[i], command=lambda v, idx=i: self.on_update(idx))
            scale.pack(fill="x")
            lbl_pulse = ttk.Label(f_slider, textvariable=self.pulse_vars[i])
            lbl_pulse.pack()
            
            # Col 2: Direction
            f_dir = ttk.Frame(row)
            f_dir.pack(side="left", padx=10)
            ttk.Label(f_dir, text="Direction:").pack()
            tk.Radiobutton(f_dir, text="Normal (1)", variable=self.dir_vars[i], value=1, command=lambda idx=i: self.on_update(idx)).pack()
            tk.Radiobutton(f_dir, text="Invert (-1)", variable=self.dir_vars[i], value=-1, command=lambda idx=i: self.on_update(idx)).pack()
            
            # Col 3: Offset
            f_off = ttk.Frame(row)
            f_off.pack(side="left", padx=10)
            ttk.Label(f_off, text="Offset (deg):").pack()
            ent_off = ttk.Entry(f_off, textvariable=self.offset_vars[i], width=8)
            ent_off.pack()
            ent_off.bind('<Return>', lambda e, idx=i: self.on_update(idx))
            ent_off.bind('<FocusOut>', lambda e, idx=i: self.on_update(idx))
            
            # Col 4: Result
            f_res = ttk.Frame(row, width=100)
            f_res.pack(side="left", padx=10)
            lbl_res_title = ttk.Label(f_res, text="Calc Angle:")
            lbl_res_title.pack()
            lbl_val = ttk.Label(f_res, text="0.0°", font=("Arial", 14, "bold"))
            lbl_val.pack()
            self.calc_angle_labels.append(lbl_val)

    def get_serial_ports(self):
        return [comport.device for comport in serial.tools.list_ports.comports()]
        
    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combo.get()
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.1)
                self.is_connected = True
                print(f"Connected to {port}")
            except Exception as e:
                print(e)
        else:
            if self.serial_port:
                self.serial_port.close()
            self.is_connected = False

    def send_servo(self, idx, pulse):
        if self.is_connected and self.serial_port:
            cmd = f"s{idx+1}-{int(pulse)}\n"
            self.serial_port.write(cmd.encode())

    def on_update(self, idx):
        # 1. Get Values
        p = self.pulse_vars[idx].get()
        d = self.dir_vars[idx].get()
        o = self.offset_vars[idx].get()
        
        # 2. Update Motor
        self.send_servo(idx, p)
        
        # 3. Calculate Angle
        # Formula: (Pulse - 1500) * Scale * Dir + Offset
        scale = 0.135
        base_angle = (p - 1500) * scale * d
        final_angle = base_angle + o
        
        # 4. Update Display
        self.calc_angle_labels[idx].config(text=f"{final_angle:.1f}°", foreground="green" if abs(final_angle) < 5 else "black")

    def save_config(self):
        # Generate new config file content
        new_dirs = [v.get() for v in self.dir_vars]
        new_offs = [v.get() for v in self.offset_vars]
        
        path = "kinematics_config.py"
        
        # Read old file to preserve comments/constants
        with open(path, "r") as f:
            lines = f.readlines()
            
        # We will brutally replace the specific lines
        # This is safe because we know the file structure
        
        final_lines = []
        for line in lines:
            if line.startswith("JOINT_DIRECTIONS"):
                final_lines.append(f"JOINT_DIRECTIONS = {new_dirs}  # Updated by calibrate.py\n")
            elif line.startswith("JOINT_OFFSETS"):
                final_lines.append(f"JOINT_OFFSETS    = {new_offs}  # Updated by calibrate.py\n")
            else:
                final_lines.append(line)
                
        with open(path, "w") as f:
            f.writelines(final_lines)
            
        messagebox.showinfo("Saved", "Configuration updated! Restart driver_gui.py to use new settings.")

if __name__ == "__main__":
    root = tk.Tk()
    app = CalibrationApp(root)
    root.mainloop()
