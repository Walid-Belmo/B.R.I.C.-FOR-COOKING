import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import serial
import serial.tools.list_ports
import time
import threading
import json
import os

# ==========================================
# CONFIGURATION
# ==========================================
BAUD_RATE = 115200
SEQUENCE_DIR = "recorded_sequences_dual"

# Servo Limits (Safety)
PULSE_MIN = 500
PULSE_MAX = 2500
NEUTRAL = 1500

# Keyboard Mapping
# ARM 1 (Left Hand)
KEY_MAP_ARM1 = {
    'q': (0, -1), # S1 Decrease
    'a': (0, 1),  # S1 Increase
    'w': (1, -1), # S2 Decrease
    's': (1, 1),  # S2 Increase
    'e': (2, -1), # S3 Decrease
    'd': (2, 1),  # S3 Increase
    'r': (3, -1), # S4 Decrease
    'f': (3, 1),  # S4 Increase
    't': (4, -1), # S5 Decrease
    'g': (4, 1)   # S5 Increase
}

# ARM 2 (Right Hand)
KEY_MAP_ARM2 = {
    'y': (0, -1), # S1 Decrease
    'h': (0, 1),  # S1 Increase
    'u': (1, -1), # S2 Decrease
    'j': (1, 1),  # S2 Increase
    'i': (2, -1), # S3 Decrease
    'k': (2, 1),  # S3 Increase
    'o': (3, -1), # S4 Decrease
    'l': (3, 1),  # S4 Increase
    'p': (4, -1), # S5 Decrease
    ';': (4, 1)   # S5 Increase (depends on keyboard layout, ; is standard US)
}

class DualArmManualControl:
    def __init__(self, root):
        self.root = root
        self.root.title("Dual Arm Manual Control (Arm 1 + Arm 2)")
        self.root.geometry("1400x900") # Wider for two panels

        # --- Connectivity ---
        self.serial_port1 = None
        self.is_connected1 = False
        self.serial_port2 = None
        self.is_connected2 = False
        
        # --- State Variables ---
        # Arm 1
        self.arm1_values = [1500] * 5
        self.arm1_vars = [tk.StringVar(value="1500") for _ in range(5)]
        self.arm1_sliders = [tk.DoubleVar(value=1500.0) for _ in range(5)]
        self.arm1_last_sent = [1500] * 5
        self.arm1_inversions = [tk.BooleanVar(value=False) for _ in range(5)]
        self.arm1_magnets = [tk.BooleanVar(value=False), tk.BooleanVar(value=False)] # M1, M2
        self.arm1_last_magnets = [False, False]

        # Arm 2
        self.arm2_values = [1500] * 5
        self.arm2_vars = [tk.StringVar(value="1500") for _ in range(5)]
        self.arm2_sliders = [tk.DoubleVar(value=1500.0) for _ in range(5)]
        self.arm2_last_sent = [1500] * 5
        self.arm2_inversions = [tk.BooleanVar(value=False) for _ in range(5)]
        # No magnets for Arm 2
        
        # Movement State
        self.active_keys = set()
        self.speed = tk.IntVar(value=5)
        
        # Recording State
        self.is_recording = False
        self.recording_start_time = 0
        self.recorded_data = [] # List of {"time": t, "arm1": [], "arm2": [], "magnets1": []}
        self.recording_name = tk.StringVar(value="dual_sequence")
        
        # Playback State
        self.available_sequences = []
        self.playlist = []
        self.is_playing = False
        self.playback_stop_event = threading.Event()

        self.setup_ui()
        
        # Key Bindings
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        
        # Control Loops
        self.control_loop()
        self.serial_loop()

    def setup_ui(self):
        # ================= TOP BAR: CONNECTION & SETTINGS =================
        top_frame = ttk.Frame(self.root)
        top_frame.pack(fill="x", padx=10, pady=5)
        
        # Arm 1 Connection
        lf1 = ttk.LabelFrame(top_frame, text="Connection: Arm 1 (Magnets)")
        lf1.pack(side="left", padx=5)
        self.port_combo1 = ttk.Combobox(lf1, values=self.get_serial_ports(), width=10)
        self.port_combo1.pack(side="left", padx=5, pady=5)
        self.btn_connect1 = ttk.Button(lf1, text="Connect", command=lambda: self.toggle_connection(1))
        self.btn_connect1.pack(side="left", padx=5)

        # Arm 2 Connection
        lf2 = ttk.LabelFrame(top_frame, text="Connection: Arm 2 (No Magnets)")
        lf2.pack(side="left", padx=5)
        self.port_combo2 = ttk.Combobox(lf2, values=self.get_serial_ports(), width=10)
        self.port_combo2.pack(side="left", padx=5, pady=5)
        self.btn_connect2 = ttk.Button(lf2, text="Connect", command=lambda: self.toggle_connection(2))
        self.btn_connect2.pack(side="left", padx=5)
        
        ttk.Button(top_frame, text="Refresh Ports", command=self.refresh_ports).pack(side="left", padx=10)

        # Settings
        sf = ttk.LabelFrame(top_frame, text="Global Settings")
        sf.pack(side="left", padx=5, fill="y")
        ttk.Label(sf, text="Speed:").pack(side="left", padx=5)
        ttk.Scale(sf, from_=1, to=50, variable=self.speed).pack(side="left", padx=5) # Removed width=100 from pack
        ttk.Label(sf, textvariable=self.speed).pack(side="left")
        ttk.Button(sf, text="Reset ALL", command=self.reset_all).pack(side="left", padx=10)

        # ================= MAIN AREA: SIDE BY SIDE PANELS =================
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # --- LEFT PANEL: ARM 1 ---
        left_panel = ttk.LabelFrame(main_frame, text="ARM 1 (Left Hand: Q,W,E,R,T + A,S,D,F,G)")
        left_panel.pack(side="left", fill="both", expand=True, padx=5)
        
        # Magnets Arm 1
        mag_f = ttk.Frame(left_panel)
        mag_f.pack(fill="x", padx=5, pady=5)
        ttk.Checkbutton(mag_f, text="Magnet 1 (Top)", variable=self.arm1_magnets[0], 
                        command=lambda: self.on_magnet_toggle(1, 0)).pack(side="left", padx=10)
        ttk.Checkbutton(mag_f, text="Magnet 2 (Side)", variable=self.arm1_magnets[1], 
                        command=lambda: self.on_magnet_toggle(1, 1)).pack(side="left", padx=10)
        
        self.create_servo_controls(left_panel, 1, self.arm1_sliders, self.arm1_vars, self.arm1_inversions,
                                   ["Q/A (Base)", "W/S (Shldr)", "E/D (Elbow)", "R/F (Wrist)", "T/G (Aux)"])

        # --- RIGHT PANEL: ARM 2 ---
        right_panel = ttk.LabelFrame(main_frame, text="ARM 2 (Right Hand: Y,U,I,O,P + H,J,K,L,;)")
        right_panel.pack(side="left", fill="both", expand=True, padx=5)
        
        ttk.Label(right_panel, text="(No Magnets Configured)").pack(pady=8)
        
        self.create_servo_controls(right_panel, 2, self.arm2_sliders, self.arm2_vars, self.arm2_inversions,
                                   ["Y/H (Base)", "U/J (Shldr)", "I/K (Elbow)", "O/L (Wrist)", "P/; (Aux)"])

        # ================= RECORDER =================
        rec_frame = ttk.LabelFrame(self.root, text="Dual Recording & Playback")
        rec_frame.pack(fill="x", padx=10, pady=5)
        
        # Recording controls
        rf1 = ttk.Frame(rec_frame)
        rf1.pack(fill="x", padx=5, pady=5)
        ttk.Label(rf1, text="Sequence Name:").pack(side="left")
        ttk.Entry(rf1, textvariable=self.recording_name).pack(side="left", padx=5)
        self.btn_record = ttk.Button(rf1, text="Start Recording", command=self.toggle_recording)
        self.btn_record.pack(side="left", padx=10)
        self.lbl_rec_status = ttk.Label(rf1, text="Ready", foreground="gray")
        self.lbl_rec_status.pack(side="left")

        # Playback controls
        rf2 = ttk.Frame(rec_frame)
        rf2.pack(fill="x", padx=5, pady=5)
        ttk.Button(rf2, text="Refresh Files", command=self.refresh_sequence_list).pack(side="left")
        
        self.combo_files = ttk.Combobox(rf2, width=30)
        self.combo_files.pack(side="left", padx=10)
        
        ttk.Button(rf2, text="Play Selected", command=self.play_single_file).pack(side="left")
        ttk.Button(rf2, text="Stop", command=self.stop_playback).pack(side="left", padx=5)

        self.refresh_sequence_list()

        # Logs
        log_frame = ttk.LabelFrame(self.root, text="Logs")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.log_text = scrolledtext.ScrolledText(log_frame, state='disabled', height=6)
        self.log_text.pack(fill="both", expand=True)

    def create_servo_controls(self, parent, arm_id, sliders, vars, inversions, labels):
        for i in range(5):
            f = ttk.Frame(parent)
            f.pack(fill="x", pady=4, padx=5)
            
            row1 = ttk.Frame(f)
            row1.pack(fill="x")
            ttk.Label(row1, text=labels[i], width=15, font=("Arial", 9, "bold")).pack(side="left")
            
            # Pass arm_id and servo_idx to callback
            s = ttk.Scale(row1, from_=PULSE_MIN, to=PULSE_MAX, variable=sliders[i], orient="horizontal",
                          command=lambda v, a=arm_id, s=i: self.on_slider_change(a, s, v))
            s.pack(side="left", fill="x", expand=True, padx=5)
            
            row2 = ttk.Frame(f)
            row2.pack(fill="x")
            e = ttk.Entry(row2, textvariable=vars[i], width=8)
            e.pack(side="left", padx=(110, 5)) # Indent to match slider start
            e.bind("<Return>", lambda event, a=arm_id, s=i: self.set_servo_entry(a, s))
            
            ttk.Button(row2, text="Set", width=4, 
                       command=lambda a=arm_id, s=i: self.set_servo_entry(a, s)).pack(side="left")
            
            ttk.Checkbutton(row2, text="Inv", variable=inversions[i]).pack(side="right")

    # ================= LOGIC: CONNECTION =================
    def get_serial_ports(self):
        return [comport.device for comport in serial.tools.list_ports.comports()]

    def refresh_ports(self):
        ports = self.get_serial_ports()
        self.port_combo1['values'] = ports
        self.port_combo2['values'] = ports

    def toggle_connection(self, arm_id):
        # Determine target
        if arm_id == 1:
            port = self.port_combo1.get()
            is_conn = self.is_connected1
            target_attr = "serial_port1"
            conn_attr = "is_connected1"
            btn = self.btn_connect1
        else:
            port = self.port_combo2.get()
            is_conn = self.is_connected2
            target_attr = "serial_port2"
            conn_attr = "is_connected2"
            btn = self.btn_connect2

        if not is_conn:
            if not port:
                self.log("Please select a port.")
                return
            try:
                ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
                setattr(self, target_attr, ser)
                setattr(self, conn_attr, True)
                btn.config(text="Disconnect")
                self.log(f"Arm {arm_id} Connected on {port}")
            except Exception as e:
                self.log(f"Connection Error Arm {arm_id}: {e}")
        else:
            ser = getattr(self, target_attr)
            if ser: ser.close()
            setattr(self, target_attr, None)
            setattr(self, conn_attr, False)
            btn.config(text="Connect")
            self.log(f"Arm {arm_id} Disconnected")

    # ================= LOGIC: CONTROL =================
    def on_slider_change(self, arm_id, servo_idx, val):
        int_val = int(float(val))
        if arm_id == 1:
            self.arm1_values[servo_idx] = int_val
            self.arm1_vars[servo_idx].set(str(int_val))
        else:
            self.arm2_values[servo_idx] = int_val
            self.arm2_vars[servo_idx].set(str(int_val))
            
        if self.is_recording:
             self.record_snapshot()

    def set_servo_entry(self, arm_id, servo_idx):
        try:
            if arm_id == 1:
                val = int(self.arm1_vars[servo_idx].get())
                target_vals = self.arm1_values
                target_sliders = self.arm1_sliders
            else:
                val = int(self.arm2_vars[servo_idx].get())
                target_vals = self.arm2_values
                target_sliders = self.arm2_sliders
            
            if val < PULSE_MIN: val = PULSE_MIN
            if val > PULSE_MAX: val = PULSE_MAX
            
            target_vals[servo_idx] = val
            target_sliders[servo_idx].set(val)
            
            if self.is_recording: self.record_snapshot()
            self.root.focus_set() # Clear focus from entry
        except:
            pass

    def on_magnet_toggle(self, arm_id, mag_idx):
        if arm_id == 1:
            state = self.arm1_magnets[mag_idx].get()
            cmd = f"m{mag_idx+1}-{1 if state else 0}\n"
            if self.is_connected1 and self.serial_port1:
                try:
                    self.serial_port1.write(cmd.encode())
                    self.arm1_last_magnets[mag_idx] = state
                    self.log(f"Arm 1 Magnet {mag_idx+1}: {state}")
                except Exception as e:
                    self.log(f"Serial Error A1 Magnet: {e}")
            if self.is_recording: self.record_snapshot()

    def reset_all(self):
        for i in range(5):
            self.arm1_values[i] = 1500
            self.arm1_sliders[i].set(1500)
            self.arm1_vars[i].set("1500")
            
            self.arm2_values[i] = 1500
            self.arm2_sliders[i].set(1500)
            self.arm2_vars[i].set("1500")
        self.log("All Servos Reset to 1500")
        if self.is_recording: self.record_snapshot()

    # ================= LOGIC: KEYBOARD =================
    def on_key_press(self, event):
        if isinstance(event.widget, (ttk.Entry, tk.Entry, ttk.Combobox)): return
        self.active_keys.add(event.char.lower())

    def on_key_release(self, event):
        if isinstance(event.widget, (ttk.Entry, tk.Entry, ttk.Combobox)): return
        char = event.char.lower()
        if char in self.active_keys:
            self.active_keys.discard(char)
            self.record_snapshot()

    def control_loop(self):
        if self.active_keys:
            step = self.speed.get()
            changed = False
            
            for key in self.active_keys:
                # ARM 1 CHECKS
                if key in KEY_MAP_ARM1:
                    idx, direction = KEY_MAP_ARM1[key]
                    if self.arm1_inversions[idx].get(): direction *= -1
                    
                    new_val = self.arm1_values[idx] + (direction * step)
                    new_val = max(PULSE_MIN, min(PULSE_MAX, new_val))
                    
                    if new_val != self.arm1_values[idx]:
                        self.arm1_values[idx] = new_val
                        self.arm1_sliders[idx].set(new_val)
                        self.arm1_vars[idx].set(str(int(new_val)))
                        changed = True

                # ARM 2 CHECKS
                if key in KEY_MAP_ARM2:
                    idx, direction = KEY_MAP_ARM2[key]
                    if self.arm2_inversions[idx].get(): direction *= -1
                    
                    new_val = self.arm2_values[idx] + (direction * step)
                    new_val = max(PULSE_MIN, min(PULSE_MAX, new_val))
                    
                    if new_val != self.arm2_values[idx]:
                        self.arm2_values[idx] = new_val
                        self.arm2_sliders[idx].set(new_val)
                        self.arm2_vars[idx].set(str(int(new_val)))
                        changed = True

            if changed and self.is_recording:
                self.record_snapshot()
        
        self.root.after(30, self.control_loop)

    def serial_loop(self):
        # Arm 1 Send
        if self.is_connected1:
            for i in range(5):
                val = int(self.arm1_values[i])
                if val != self.arm1_last_sent[i]:
                    try:
                        self.serial_port1.write(f"s{i+1}-{val}\n".encode())
                        self.arm1_last_sent[i] = val
                    except: pass
        
        # Arm 2 Send
        if self.is_connected2:
            for i in range(5):
                val = int(self.arm2_values[i])
                if val != self.arm2_last_sent[i]:
                    try:
                        self.serial_port2.write(f"s{i+1}-{val}\n".encode())
                        self.arm2_last_sent[i] = val
                    except: pass

        self.root.after(100, self.serial_loop)

    # ================= LOGIC: RECORDING =================
    def toggle_recording(self):
        if not self.is_recording:
            self.is_recording = True
            self.recording_start_time = time.time()
            self.recorded_data = []
            self.record_snapshot(force=True)
            self.btn_record.config(text="Stop Recording")
            self.lbl_rec_status.config(text="RECORDING...", foreground="red")
            self.log("Recording Started")
        else:
            self.is_recording = False
            self.save_sequence()
            self.btn_record.config(text="Start Recording")
            self.lbl_rec_status.config(text="Saved", foreground="green")
            self.log("Recording Stopped & Saved")

    def record_snapshot(self, force=False):
        if not self.is_recording: return
        
        t = round(time.time() - self.recording_start_time, 3)
        vals1 = [int(v) for v in self.arm1_values]
        vals2 = [int(v) for v in self.arm2_values]
        mags1 = [self.arm1_magnets[0].get(), self.arm1_magnets[1].get()]
        
        # Check against last
        if not force and self.recorded_data:
            last = self.recorded_data[-1]
            if (last["arm1"] == vals1 and 
                last["arm2"] == vals2 and 
                last["magnets1"] == mags1):
                return
        
        self.recorded_data.append({
            "time": t,
            "arm1": vals1,
            "arm2": vals2,
            "magnets1": mags1
        })

    def save_sequence(self):
        msg = {
            "name": self.recording_name.get(),
            "created": time.ctime(),
            "start_arm1": self.recorded_data[0]["arm1"] if self.recorded_data else self.arm1_values,
            "start_arm2": self.recorded_data[0]["arm2"] if self.recorded_data else self.arm2_values,
            "timeline": self.recorded_data
        }
        
        if not os.path.exists(SEQUENCE_DIR): os.makedirs(SEQUENCE_DIR)
        
        # Unique filename
        base = self.recording_name.get().strip() or "dual_seq"
        path = os.path.join(SEQUENCE_DIR, f"{base}.json")
        c = 1
        while os.path.exists(path):
            path = os.path.join(SEQUENCE_DIR, f"{base}_{c}.json")
            c += 1
            
        with open(path, 'w') as f:
            json.dump(msg, f, indent=2)
        
        self.refresh_sequence_list()

    def refresh_sequence_list(self):
        self.combo_files['values'] = []
        if os.path.exists(SEQUENCE_DIR):
            files = [f for f in os.listdir(SEQUENCE_DIR) if f.endswith(".json")]
            self.combo_files['values'] = files

    def stop_playback(self):
        self.playback_stop_event.set()
        self.is_playing = False

    def play_single_file(self):
        fname = self.combo_files.get()
        if not fname: return
        
        if self.is_playing: return
        
        path = os.path.join(SEQUENCE_DIR, fname)
        if not os.path.exists(path): return

        t = threading.Thread(target=self.run_playback, args=(path,))
        t.daemon = True
        t.start()

    def run_playback(self, path):
        self.is_playing = True
        self.playback_stop_event.clear()
        self.log(f"Playing {os.path.basename(path)}")
        
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            
            timeline = data.get("timeline", [])
            start_time = time.time()
            
            for point in timeline:
                if self.playback_stop_event.is_set(): break
                
                target_t = point["time"]
                
                while (time.time() - start_time) < target_t:
                    if self.playback_stop_event.is_set(): break
                    time.sleep(0.01)
                
                # Apply Values
                a1 = point["arm1"]
                a2 = point["arm2"]
                m1 = point.get("magnets1", [False, False])
                
                # Update UI vars (thread safe enough for variables usually, but better to schedule)
                # We update memory state immediately for the serial loop to pick up
                
                for i in range(5):
                    self.arm1_values[i] = a1[i]
                    self.arm2_values[i] = a2[i]
                
                # Magnets need direct send or check in serial loop. 
                # Since we don't hold "state" for magnets in serial loop same way (we only send on toggle),
                # we should send command here if changed.
                
                # Simple check
                if m1 != self.arm1_last_magnets:
                    # Update UI
                    self.arm1_magnets[0].set(m1[0])
                    self.arm1_magnets[1].set(m1[1])
                    # Send
                    self.on_magnet_toggle(1, 0)
                    self.on_magnet_toggle(1, 1)

                # Update UI visuals
                self.root.after(0, lambda v1=a1, v2=a2: self.update_playback_ui(v1, v2))
                
        except Exception as e:
            self.log(f"Playback Error: {e}")
            
        self.is_playing = False
        self.log("Playback Finished")

    def update_playback_ui(self, v1, v2):
        for i in range(5):
            self.arm1_sliders[i].set(v1[i])
            self.arm1_vars[i].set(str(v1[i]))
            self.arm2_sliders[i].set(v2[i])
            self.arm2_vars[i].set(str(v2[i]))

    def log(self, msg):
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"> {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

if __name__ == "__main__":
    root = tk.Tk()
    app = DualArmManualControl(root)
    root.mainloop()
