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
SEQUENCE_DIR = "recorded_sequences"

# Servo Limits (Safety)
PULSE_MIN = 500
PULSE_MAX = 2500
NEUTRAL = 1500

# Keyboard Mapping
KEY_MAP = {
    'q': (0, -1), # Servo 1 Decrease
    'a': (0, 1),  # Servo 1 Increase
    'w': (1, -1), # Servo 2 Decrease
    's': (1, 1),  # Servo 2 Increase
    'e': (2, -1), # Servo 3 Decrease
    'd': (2, 1),  # Servo 3 Increase
    'r': (3, -1), # Servo 4 Decrease
    'f': (3, 1),  # Servo 4 Increase
    't': (4, -1), # Servo 5 Decrease
    'g': (4, 1)   # Servo 5 Increase
}

class RobotManualControl:
    def __init__(self, root):
        self.root = root
        self.root.title("Manual Robot Arm Control & Recorder")
        self.root.geometry("800x800")

        self.serial_port = None
        self.is_connected = False
        
        # State Variables
        self.servo_values = [1500] * 5 # Current pulse width - Floats for slider precision
        self.servo_vars = [tk.StringVar(value="1500") for _ in range(5)] # For Entry/Label
        self.slider_vars = [tk.DoubleVar(value=1500.0) for _ in range(5)] # For Sliders
        
        self.last_sent_values = [1500] * 5
        self.inversions = [tk.BooleanVar(value=False) for _ in range(5)] # True = Inverted
        
        # Magnet State (False = OFF, True = ON)
        self.magnet_states = [tk.BooleanVar(value=False), tk.BooleanVar(value=False)] 
        self.last_sent_magnets = [False, False]

        # Movement State
        self.active_keys = set() # Keys currently pressed
        self.speed = tk.IntVar(value=5) # Step size per tick (higher = faster)
        
        # Recording State
        self.is_recording = False
        self.recording_start_time = 0
        self.recorded_data = [] # List of {"time": t, "values": [v1...v5]}
        self.recording_name = tk.StringVar(value="my_sequence")
        
        # Playback State
        self.available_sequences = []
        self.playlist = []
        self.is_playing = False
        self.playback_stop_event = threading.Event()

        self.setup_ui()
        
        # Key Bindings
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        
        # Start Control Loops
        self.control_loop()
        self.serial_loop()

    def setup_ui(self):
        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        self.port_combo = ttk.Combobox(conn_frame, values=self.get_serial_ports())
        self.port_combo.pack(side="left", padx=5, pady=5)
        
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5)
        
        self.btn_refresh = ttk.Button(conn_frame, text="Refresh", command=lambda: self.port_combo.config(values=self.get_serial_ports()))
        self.btn_refresh.pack(side="left", padx=5)

        # --- Settings Frame ---
        settings_frame = ttk.LabelFrame(self.root, text="Control Settings")
        settings_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(settings_frame, text="Speed (Step Size):").pack(side="left", padx=5)
        ttk.Scale(settings_frame, from_=1, to=50, variable=self.speed, orient="horizontal").pack(side="left", fill="x", expand=True, padx=5)
        ttk.Label(settings_frame, textvariable=self.speed).pack(side="left", padx=5)
        
        ttk.Separator(settings_frame, orient="vertical").pack(side="left", fill="y", padx=10, pady=2)
        ttk.Button(settings_frame, text="Reset All (1500)", command=self.reset_to_neutral).pack(side="left", padx=5)

        # --- Magnet Control Frame ---
        mag_frame = ttk.LabelFrame(self.root, text="Electromagnets")
        mag_frame.pack(fill="x", padx=10, pady=5)
        
        # Magnet 1
        m1_btn = ttk.Checkbutton(mag_frame, text="Magnet 1 (Top/Gravity)", variable=self.magnet_states[0], 
                                 command=lambda: self.on_magnet_toggle(0))
        m1_btn.pack(side="left", padx=20)

        # Magnet 2
        m2_btn = ttk.Checkbutton(mag_frame, text="Magnet 2 (Side/Tip)", variable=self.magnet_states[1], 
                                 command=lambda: self.on_magnet_toggle(1))
        m2_btn.pack(side="left", padx=20)

        # --- Servo Status Frame ---
        status_frame = ttk.LabelFrame(self.root, text="Servo Status & Controls")
        status_frame.pack(fill="x", padx=10, pady=5)

        keys_info = [
            ("Servo 1 (Base)", "Q / A"),
            ("Servo 2 (Shoulder)", "W / S"),
            ("Servo 3 (Elbow)", "E / D"),
            ("Servo 4 (Wrist)", "R / F"),
            ("Servo 5 (Aux)", "T / G"),
        ]

        for i in range(5):
            f = ttk.Frame(status_frame)
            f.pack(fill="x", pady=2)
            
            # Row 1: Name, Slider
            row1 = ttk.Frame(f)
            row1.pack(fill="x")
            
            # Name & Keys
            ttk.Label(row1, text=f"{keys_info[i][0]} [{keys_info[i][1]}]", width=25, anchor="w").pack(side="left", padx=5)
            
            # Slider
            scale = ttk.Scale(row1, from_=PULSE_MIN, to=PULSE_MAX, variable=self.slider_vars[i], orient="horizontal", command=lambda v, idx=i: self.on_slider_change(idx, v))
            scale.pack(side="left", fill="x", expand=True, padx=5)

            # Row 2: Entry, Set, Invert
            row2 = ttk.Frame(f)
            row2.pack(fill="x", pady=2)
            
            # Spacer to align with slider start (approx)
            ttk.Frame(row2, width=170).pack(side="left") 

            # Value Entry
            entry_val = ttk.Entry(row2, textvariable=self.servo_vars[i], width=8, font=("Consolas", 10))
            entry_val.pack(side="left", padx=5)
            entry_val.bind("<Return>", lambda event, idx=i: self.set_servo_from_entry(idx))

            # Set Button
            btn_set = ttk.Button(row2, text="Set", width=4, command=lambda idx=i: self.set_servo_from_entry(idx))
            btn_set.pack(side="left", padx=2)
            
            # Invert Checkbox
            cb = ttk.Checkbutton(row2, text="Invert", variable=self.inversions[i])
            cb.pack(side="right", padx=10)

        # --- Recording Frame ---
        rec_frame = ttk.LabelFrame(self.root, text="Recording")
        rec_frame.pack(fill="x", padx=10, pady=5)
        
        ttk.Label(rec_frame, text="Name:").pack(side="left", padx=5)
        ttk.Entry(rec_frame, textvariable=self.recording_name, width=20).pack(side="left", padx=5)
        
        self.btn_record = ttk.Button(rec_frame, text="Start Recording", command=self.toggle_recording)
        self.btn_record.pack(side="left", padx=10)
        
        self.lbl_rec_status = ttk.Label(rec_frame, text="Ready", foreground="gray")
        self.lbl_rec_status.pack(side="left", padx=5)

        # --- Playback & Playlist Frame ---
        play_frame = ttk.LabelFrame(self.root, text="Playback & Playlist")
        play_frame.pack(fill="x", padx=10, pady=5)
        
        # Top Controls: Refresh, Play All
        pf_ctrl = ttk.Frame(play_frame)
        pf_ctrl.pack(fill="x", padx=5, pady=2)
        ttk.Button(pf_ctrl, text="Refresh Files", command=self.refresh_sequence_list).pack(side="left", padx=2)
        ttk.Button(pf_ctrl, text="Safe Play Playlist", command=self.start_playlist_thread).pack(side="right", padx=2)
        
        # Dual Listbox (Available -> Playlist)
        pf_lists = ttk.Frame(play_frame)
        pf_lists.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Left: Available
        lf = ttk.LabelFrame(pf_lists, text="Available Sequences")
        lf.pack(side="left", fill="both", expand=True)
        self.lst_available = tk.Listbox(lf, height=6)
        self.lst_available.pack(fill="both", expand=True, padx=2, pady=2)
        
        # Buttons: Add/Remove
        bf = ttk.Frame(pf_lists)
        bf.pack(side="left", padx=5)
        ttk.Button(bf, text="Add >>", command=self.add_to_playlist).pack(pady=2)
        ttk.Button(bf, text="<< Del", command=self.remove_from_playlist).pack(pady=2)
        ttk.Separator(bf, orient="horizontal").pack(fill="x", pady=5)
        ttk.Button(bf, text="Up", command=self.move_playlist_up).pack(pady=2)
        ttk.Button(bf, text="Down", command=self.move_playlist_down).pack(pady=2)

        # Right: Playlist
        rf = ttk.LabelFrame(pf_lists, text="Execution Playlist")
        rf.pack(side="left", fill="both", expand=True)
        self.lst_playlist = tk.Listbox(rf, height=6)
        self.lst_playlist.pack(fill="both", expand=True, padx=2, pady=2)
        
        self.refresh_sequence_list()

        # --- Log Frame ---
        log_frame = ttk.LabelFrame(self.root, text="Logs")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, state='disabled', height=10)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Instructions
        instr_lbl = ttk.Label(self.root, text="Hold keys to move servos. Ensure caps lock is OFF.", foreground="gray")
        instr_lbl.pack(pady=5)

    def toggle_recording(self):
        if not self.is_recording:
            # Start
            self.is_recording = True
            self.recording_start_time = time.time()
            self.recorded_data = []
            
            # Capture initial state
            self.record_snapshot()
            
            self.btn_record.config(text="Stop Recording")
            self.lbl_rec_status.config(text="Recording...", foreground="red")
            self.log_message("Recording STARTED")
        else:
            # Stop & Save
            self.is_recording = False
            self.btn_record.config(text="Start Recording")
            self.lbl_rec_status.config(text="Saved", foreground="green")
            self.save_sequence()
            self.log_message("Recording STOPPED")

    def record_snapshot(self):
        if not self.is_recording: return
        
        t = time.time() - self.recording_start_time
        vals = [self.servo_values[i] for i in range(5)]
        
        # Get magnet states as booleans/ints
        mags = [self.magnet_states[0].get(), self.magnet_states[1].get()]
        
        # Only record if changed or first point
        # Compare current state with LAST recorded state
        current_state = {"values": vals, "magnets": mags}
        
        should_record = False
        if not self.recorded_data:
            should_record = True
        else:
            last = self.recorded_data[-1]
            if last["values"] != vals or last.get("magnets", [False, False]) != mags:
                should_record = True
                
        if should_record:
            self.recorded_data.append({
                "time": round(t, 3), # ms precision
                "values": vals,
                "magnets": mags # Save magnets too
            })

    def save_sequence(self):
        base_name = self.recording_name.get().strip()
        if not base_name: base_name = "untitled"
        
        if not os.path.exists(SEQUENCE_DIR):
            os.makedirs(SEQUENCE_DIR)
            
        # Auto-increment to prevent overwrite
        filename = f"{base_name}.json"
        filepath = os.path.join(SEQUENCE_DIR, filename)
        
        counter = 1
        while os.path.exists(filepath):
            filename = f"{base_name}_{counter}.json"
            filepath = os.path.join(SEQUENCE_DIR, filename)
            counter += 1
        
        # Save payload
        # Current magnets state for start condition
        start_mags = [self.magnet_states[0].get(), self.magnet_states[1].get()]
        if self.recorded_data:
             start_mags = self.recorded_data[0].get("magnets", [False, False])

        payload = {
            "name": filename.replace(".json", ""), # Use the actual unique name
            "created": time.ctime(),
            "start_position": self.recorded_data[0]["values"] if self.recorded_data else [1500]*5,
            "start_magnets": start_mags,
            "timeline": self.recorded_data
        }
        
        try:
            with open(filepath, 'w') as f:
                json.dump(payload, f, indent=2)
            self.log_message(f"Saved sequence to {filename}")
            self.refresh_sequence_list()
            
            # Update the UI name to the next likely sequence name to avoid confusion
            if counter > 1:
                self.recording_name.set(f"{base_name}_{counter}")
            else:
                self.recording_name.set(f"{base_name}_1")
                
        except Exception as e:
            self.log_message(f"Error saving: {e}")

    # --- Playlist Management ---
    def refresh_sequence_list(self):
        self.lst_available.delete(0, tk.END)
        self.available_sequences = []
        if os.path.exists(SEQUENCE_DIR):
            for f in os.listdir(SEQUENCE_DIR):
                if f.endswith(".json"):
                    self.lst_available.insert(tk.END, f)
                    self.available_sequences.append(f)

    def add_to_playlist(self):
        sel = self.lst_available.curselection()
        if not sel: return
        fname = self.available_sequences[sel[0]]
        self.playlist.append(fname)
        self.lst_playlist.insert(tk.END, fname)

    def remove_from_playlist(self):
        sel = self.lst_playlist.curselection()
        if not sel: return
        idx = sel[0]
        self.playlist.pop(idx)
        self.lst_playlist.delete(idx)

    def move_playlist_up(self):
        sel = self.lst_playlist.curselection()
        if not sel: return
        idx = sel[0]
        if idx > 0:
            item = self.playlist.pop(idx)
            self.playlist.insert(idx-1, item)
            self.rebuild_playlist_ui()
            self.lst_playlist.selection_set(idx-1)

    def move_playlist_down(self):
        sel = self.lst_playlist.curselection()
        if not sel: return
        idx = sel[0]
        if idx < len(self.playlist) - 1:
            item = self.playlist.pop(idx)
            self.playlist.insert(idx+1, item)
            self.rebuild_playlist_ui()
            self.lst_playlist.selection_set(idx+1)

    def rebuild_playlist_ui(self):
        self.lst_playlist.delete(0, tk.END)
        for item in self.playlist:
            self.lst_playlist.insert(tk.END, item)

    # --- Playback Logic ---
    def start_playlist_thread(self):
        if not self.playlist:
            self.log_message("Playlist is empty!")
            return
        if self.is_playing:
            self.log_message("Already playing!")
            return
        
        self.is_playing = True
        self.playback_stop_event.clear()
        
        t = threading.Thread(target=self.run_playlist)
        t.daemon = True
        t.start()

    def run_playlist(self):
        self.log_message("=== Starting Ecosystem Playback ===")
        
        try:
            for i, fname in enumerate(self.playlist):
                if self.playback_stop_event.is_set(): break
                
                self.log_message(f"Preparing: {fname} ({i+1}/{len(self.playlist)})")
                
                # Load file
                path = os.path.join(SEQUENCE_DIR, fname)
                with open(path, 'r') as f:
                    seq_data = json.load(f)
                
                start_pos = seq_data.get("start_position", [1500]*5)
                timeline = seq_data.get("timeline", [])
                
                # 1. Check Initial Position (Strict Verification)
                current_server_vals = self.servo_values[:] # Copy
                if not self.verify_start_position(current_server_vals, start_pos, fname):
                    break
                
                # 2. Execute Timeline
                self.log_message(f"Playing Sequence: {seq_data['name']}")
                start_time = time.time()
                
                # Keep track of last sent magnet state to avoid spamming serial
                last_mags_playback = [False, False] 
                
                for point in timeline:
                    if self.playback_stop_event.is_set(): break
                    
                    target_time = point["time"]
                    values = point["values"]
                    magnets = point.get("magnets", [False, False]) # Default OFF for old files
                    
                    # Wait until it's time
                    while (time.time() - start_time) < target_time:
                         if self.playback_stop_event.is_set(): break
                         time.sleep(0.01)
                    
                    # Update Servos
                    for s_idx in range(5):
                        self.servo_values[s_idx] = values[s_idx]
                        
                    # Update Magnets (Direct Send)
                    if magnets != last_mags_playback:
                         self.send_magnet_command(0, magnets[0])
                         self.send_magnet_command(1, magnets[1])
                         last_mags_playback = magnets[:]
                         
                         # Update UI variables
                         self.magnet_states[0].set(magnets[0])
                         self.magnet_states[1].set(magnets[1])

                    # Update sliders visually
                    self.root.after(0, lambda v=values: self.update_sliders_from_playback(v))

            self.log_message("=== Playlist Finished ===")
            
        except Exception as e:
            self.log_message(f"Playback Error: {e}")
        
        self.is_playing = False

    def send_magnet_command(self, idx, state):
         if self.is_connected and self.serial_port:
             cmd = f"m{idx+1}-{1 if state else 0}\n"
             try:
                self.serial_port.write(cmd.encode())
             except: pass

    def update_sliders_from_playback(self, values):
        for i in range(5):
             self.slider_vars[i].set(values[i])
             self.servo_vars[i].set(str(values[i]))

    def verify_start_position(self, current, target, seq_name):
        """Checks if current position matches target within tolerance. Returns True if OK."""
        TOLERANCE = 10 # microseconds
        
        mismatch = False
        for i in range(5):
            if abs(current[i] - target[i]) > TOLERANCE:
                mismatch = True
                break
        
        if mismatch:
            err_msg = f"Position mismatch for '{seq_name}'!\n\nRobot: {current}\nExpected: {target}"
            self.log_message(f"ERROR: {err_msg}")
            self.root.after(0, lambda: messagebox.showerror("Playback Error", err_msg))
            return False
            
        return True

    def move_to_position_safely(self, current, target):
        """Linearly interpolates from current to target over 2 seconds to avoid jumps"""
        # (This function is kept for reference but no longer used in standard playback)
        self.log_message("Transitioning to start position...")
        steps = 50
        duration = 2.0
        dt = duration / steps
        
        for step in range(steps):
            if self.playback_stop_event.is_set(): return False
            
            t = step / float(steps)
            interp_vals = []
            for i in range(5):
                val = current[i] + (target[i] - current[i]) * t
                interp_vals.append(int(val))
            
            # Update system
            for i in range(5):
                self.servo_values[i] = interp_vals[i]
            
            self.root.after(0, lambda v=interp_vals: self.update_sliders_from_playback(v))
            time.sleep(dt)
            
        return True

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

    def reset_to_neutral(self):
        for i in range(5):
            self.servo_values[i] = 1500
            self.servo_vars[i].set("1500")
            self.slider_vars[i].set(1500.0)
        
        # Optionally reset magnets too?
        # self.magnet_states[0].set(False)
        # self.magnet_states[1].set(False)
        # self.on_magnet_toggle(0)
        # self.on_magnet_toggle(1)
        
        self.log_message("All servos reset to 1500")
        
        if self.is_recording:
             self.record_snapshot()

    def on_magnet_toggle(self, idx):
        # 1. Send Command Immediately
        state = self.magnet_states[idx].get()
        cmd = f"m{idx+1}-{1 if state else 0}\n"
        if self.is_connected and self.serial_port:
             try:
                self.serial_port.write(cmd.encode())
                self.last_sent_magnets[idx] = state
                self.log_message(f"Magnet {idx+1} {'ON' if state else 'OFF'}")
             except Exception as e:
                self.log_message(f"Serial Error: {e}")
        
        # 2. Record if recording
        if self.is_recording:
            self.record_snapshot()

    def log_message(self, msg):
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"> {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

    def on_slider_change(self, idx, val):
        # Update internal value from slider
        int_val = int(float(val))
        self.servo_values[idx] = int_val
        self.servo_vars[idx].set(str(int_val))
        # No need to send here, serial_loop handles it
        
        # We don't record every micro-move of the slider, but we could sample periodically
        # Or just rely on the key-release/end of drag equivalent?
        # For now, let's just record if recording is on, but maybe debounced?
        # Actually simplest is just record:
        if self.is_recording:
             # Limit recording frequency if needed, but current implementation checks for changes
             self.record_snapshot()

    def set_servo_from_entry(self, idx):
        try:
            val_str = self.servo_vars[idx].get()
            val = int(val_str)
            
            # Clamp
            if val < PULSE_MIN: val = PULSE_MIN
            if val > PULSE_MAX: val = PULSE_MAX
            
            # Update internal state
            self.servo_values[idx] = val
            self.slider_vars[idx].set(val) # Sync slider
            self.servo_vars[idx].set(str(val))
            self.log_message(f"Servo {idx+1} set to {val}")
            
            # Record change if recording
            self.record_snapshot()

            # Refocus root to allow immediate key control
            self.root.focus_set()
            
        except ValueError:
            self.log_message(f"Invalid input for Servo {idx+1}")
            self.servo_vars[idx].set(str(int(self.servo_values[idx])))

    def on_key_press(self, event):
        # Ignore key presses if typing in an Entry widget
        if isinstance(event.widget, (ttk.Entry, tk.Entry)):
            return

        char = event.char.lower()
        if char in KEY_MAP:
            self.active_keys.add(char)

    def on_key_release(self, event):
        # Ignore key releases if typing in an Entry widget
        if isinstance(event.widget, (ttk.Entry, tk.Entry)):
            return

        char = event.char.lower()
        if char in KEY_MAP:
            self.active_keys.discard(char)
            # Record final position after key release
            self.record_snapshot()

    def control_loop(self):
        """ Calculates servo positions based on key presses. High frequency for smoothness. """
        if self.active_keys:
            step = self.speed.get()

            # Process all active keys
            for key in self.active_keys:
                servo_idx, direction = KEY_MAP[key]
                
                # Apply inversion if checked
                if self.inversions[servo_idx].get():
                    direction *= -1

                new_value = self.servo_values[servo_idx] + (direction * step)
                
                # Clamp values
                if new_value < PULSE_MIN: new_value = PULSE_MIN
                if new_value > PULSE_MAX: new_value = PULSE_MAX
                
                if new_value != self.servo_values[servo_idx]:
                    self.servo_values[servo_idx] = new_value
                    self.servo_vars[servo_idx].set(str(int(new_value)))
                    self.slider_vars[servo_idx].set(new_value) # Sync slider
                    
            if self.is_recording:
                self.record_snapshot()

        self.root.after(30, self.control_loop) # ~33Hz calculation rate

    def serial_loop(self):
        """ Sends commands to the robot. Lower frequency to prevent Serial Buffer Flooding. """
        if self.is_connected:
            self.send_servo_updates()
        
        # 10 Hz (100ms) is safer for the ESP32 that prints verbose output
        self.root.after(100, self.serial_loop)

    def send_servo_updates(self):
        # Send only the LATEST calculated value
        for i in range(5):
            val = int(self.servo_values[i])
            # Only send if the value has changed since LAST TRANSMISSION
            if val != self.last_sent_values[i]:
                try:
                    cmd = f"s{i+1}-{val}\n"
                    self.serial_port.write(cmd.encode())
                    self.last_sent_values[i] = val
                except Exception as e:
                    print(f"Serial Error: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotManualControl(root)
    root.mainloop()
