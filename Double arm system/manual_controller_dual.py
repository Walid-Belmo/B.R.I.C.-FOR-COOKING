import tkinter as tk
from tkinter import ttk, scrolledtext, simpledialog, messagebox
import serial
import serial.tools.list_ports
import time
import threading
import json
import os
import shutil

# ==========================================
# CONFIGURATION
# ==========================================
BAUD_RATE = 115200
BASE_REC_DIR = "recorded_sequences_dual"

PULSE_MIN = 500
PULSE_MAX = 2500
NEUTRAL = 1500

# Keyboard Mapping
KEY_MAP_ARM1 = {
    'q': (0, -1), 'a': (0, 1), # Base
    'w': (1, -1), 's': (1, 1), # Shoulder
    'e': (2, -1), 'd': (2, 1), # Elbow
    'r': (3, -1), 'f': (3, 1), # Wrist
    't': (4, -1), 'g': (4, 1)  # Aux
}

KEY_MAP_ARM2 = {
    'y': (0, -1), 'h': (0, 1), 
    'u': (1, -1), 'j': (1, 1),
    'i': (2, -1), 'k': (2, 1),
    'o': (3, -1), 'l': (3, 1),
    'p': (4, -1), ';': (4, 1)
}

class DualArmManualControl:
    def __init__(self, root):
        self.root = root
        self.root.title("Dual Arm Director's Console")
        self.root.geometry("1500x950")

        # --- Connectivity ---
        self.serial_port1 = None
        self.is_connected1 = False
        self.serial_port2 = None
        self.is_connected2 = False
        
        # --- Servo State ---
        self.arm1_values = [1500] * 5
        self.arm2_values = [1500] * 5
        
        # UI vars
        self.arm1_vars = [tk.StringVar(value="1500") for _ in range(5)]
        self.arm1_sliders = [tk.DoubleVar(value=1500.0) for _ in range(5)]
        self.arm1_inversions = [tk.BooleanVar(value=False) for _ in range(5)]
        
        self.arm2_vars = [tk.StringVar(value="1500") for _ in range(5)]
        self.arm2_sliders = [tk.DoubleVar(value=1500.0) for _ in range(5)]
        self.arm2_inversions = [tk.BooleanVar(value=False) for _ in range(5)]

        self.arm1_magnets = [tk.BooleanVar(value=False), tk.BooleanVar(value=False)] 
        self.arm1_last_magnets = [False, False]

        self.arm1_last_sent = [1500] * 5
        self.arm2_last_sent = [1500] * 5

        # --- Scene & Recording State ---
        self.current_scene = tk.StringVar(value="")
        self.step_name = tk.StringVar(value="")
        
        self.is_recording = False
        self.rec_start_time = 0
        self.rec_data = [] # Timeline
        
        # --- Playback ---
        self.playlist = [] # List of full paths
        self.is_playing = False
        self.stop_event = threading.Event()

        # Input Handling
        self.active_keys = set()
        self.speed = tk.IntVar(value=5)

        self.ensure_dirs()
        self.setup_ui()
        
        # Bindings & Loops
        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.control_loop()
        self.serial_loop()

    def ensure_dirs(self):
        if not os.path.exists(BASE_REC_DIR):
            os.makedirs(BASE_REC_DIR)

    # ================= UI SETUP =================
    def setup_ui(self):
        # 1. TOP BAR (Connections)
        top = ttk.Frame(self.root)
        top.pack(fill="x", padx=10, pady=5)
        
        self.create_conn_box(top, "Arm 1 (Left/Magnets)", 1)
        self.create_conn_box(top, "Arm 2 (Right)", 2)
        
        ttk.Label(top, text="Speed:").pack(side="left", padx=5)
        ttk.Scale(top, from_=1, to=50, variable=self.speed).pack(side="left", padx=5)
        ttk.Button(top, text="Reset All", command=self.reset_all).pack(side="left", padx=20)

        # 2. MAIN SPLIT (Controls vs Director)
        content = ttk.Frame(self.root)
        content.pack(fill="both", expand=True, padx=10, pady=5)
        
        # LEFT: Servo Controls
        ctrl_pane = ttk.PanedWindow(content, orient="horizontal")
        ctrl_pane.pack(side="top", fill="both", expand=True)

        left_c = ttk.LabelFrame(ctrl_pane, text="Arm 1 Controls (QWERTY)")
        ctrl_pane.add(left_c, weight=1)
        
        # Magnets
        mf = ttk.Frame(left_c)
        mf.pack(fill="x", pady=5)
        ttk.Checkbutton(mf, text="Mag 1 (Top)", variable=self.arm1_magnets[0], command=lambda: self.update_magnets(True)).pack(side="left", padx=10)
        ttk.Checkbutton(mf, text="Mag 2 (Side)", variable=self.arm1_magnets[1], command=lambda: self.update_magnets(True)).pack(side="left", padx=10)
        
        self.build_servo_sliders(left_c, 1)

        right_c = ttk.LabelFrame(ctrl_pane, text="Arm 2 Controls (UIOP)")
        ctrl_pane.add(right_c, weight=1)
        ttk.Label(right_c, text="(No Magnets)").pack(pady=5)
        self.build_servo_sliders(right_c, 2)
        
        # BOTTOM: Director's Panel (Scene & Recording)
        director = ttk.LabelFrame(self.root, text="Director's Studio")
        director.pack(fill="both", expand=True, padx=10, pady=5)
        
        # --- Scene Selector ---
        scene_frame = ttk.Frame(director)
        scene_frame.pack(fill="x", padx=5, pady=5)
        ttk.Label(scene_frame, text="SCENE:").pack(side="left")
        self.combo_scenes = ttk.Combobox(scene_frame, textvariable=self.current_scene, state="readonly", width=30)
        self.combo_scenes.pack(side="left", padx=5)
        self.combo_scenes.bind("<<ComboboxSelected>>", self.on_scene_selected)
        
        ttk.Button(scene_frame, text="New Scene", command=self.create_scene).pack(side="left", padx=5)
        ttk.Button(scene_frame, text="Refresh", command=self.refresh_scenes).pack(side="left")

        # --- 3 Columns: Available Steps | Properties | Playlist ---
        cols = ttk.Frame(director)
        cols.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Col 1: Available Files in Scene
        f1 = ttk.LabelFrame(cols, text="1. Sequence Steps")
        f1.pack(side="left", fill="both", expand=True, padx=2)
        
        rec_row = ttk.Frame(f1)
        rec_row.pack(fill="x", pady=2)
        ttk.Label(rec_row, text="Step Name:").pack(side="left")
        ttk.Entry(rec_row, textvariable=self.step_name).pack(side="left", fill="x", expand=True, padx=5)
        self.btn_rec = ttk.Button(rec_row, text="REC", command=self.toggle_rec)
        self.btn_rec.pack(side="left")
        self.lbl_rec = ttk.Label(rec_row, text="⚫", foreground="gray")
        self.lbl_rec.pack(side="left", padx=2)

        self.list_files = tk.Listbox(f1)
        self.list_files.pack(fill="both", expand=True)
        self.list_files.bind("<<ListboxSelect>>", self.on_file_select)
        
        ttk.Button(f1, text="Add to Playlist >>", command=self.add_to_playlist).pack(fill="x", pady=2)

        # Col 2: Properties
        f2 = ttk.LabelFrame(cols, text="2. Sequence Info")
        f2.pack(side="left", fill="both", expand=True, padx=2)
        self.txt_props = tk.Text(f2, width=30, height=8, state="disabled", bg="#f0f0f0")
        self.txt_props.pack(fill="both", expand=True)

        # Col 3: Playlist
        f3 = ttk.LabelFrame(cols, text="3. Master Playlist")
        f3.pack(side="left", fill="both", expand=True, padx=2)
        
        self.list_playlist = tk.Listbox(f3)
        self.list_playlist.pack(fill="both", expand=True)
        
        btns = ttk.Frame(f3)
        btns.pack(fill="x")
        ttk.Button(btns, text="Remove", command=self.rem_from_playlist).pack(side="left", expand=True)
        ttk.Button(btns, text="Clear", command=lambda: self.list_playlist.delete(0,tk.END) or self.playlist.clear()).pack(side="left", expand=True)
        
        play_area = ttk.Frame(f3)
        play_area.pack(fill="x", pady=5)
        ttk.Button(play_area, text="▶ PLAY SCENE", command=self.play_playlist).pack(fill="x")
        ttk.Button(play_area, text="⏹ STOP", command=self.stop_playback).pack(fill="x")

        # Init
        self.refresh_scenes()

    def create_conn_box(self, parent, title, idx):
        lf = ttk.LabelFrame(parent, text=title)
        lf.pack(side="left", padx=5)
        cb = ttk.Combobox(lf, width=10, values=self.get_ports())
        cb.pack(side="left", padx=2)
        btn = ttk.Button(lf, text="Connect", command=lambda: self.toggle_conn(idx, cb))
        btn.pack(side="left", padx=2)
        if idx == 1:
            self.cb_p1 = cb
            self.btn_p1 = btn
        else:
            self.cb_p2 = cb
            self.btn_p2 = btn

    def build_servo_sliders(self, parent, arm_id):
        sliders = self.arm1_sliders if arm_id == 1 else self.arm2_sliders
        vars = self.arm1_vars if arm_id == 1 else self.arm2_vars
        invs = self.arm1_inversions if arm_id == 1 else self.arm2_inversions
        
        labels = ["Base", "Shoulder", "Elbow", "Wrist", "Aux"]
        for i in range(5):
            row = ttk.Frame(parent)
            row.pack(fill="x", padx=5, pady=2)
            ttk.Label(row, text=f"{labels[i]}:", width=8).pack(side="left")
            s = ttk.Scale(row, from_=PULSE_MIN, to=PULSE_MAX, variable=sliders[i], orient="horizontal",
                          command=lambda v, a=arm_id, idx=i: self.on_slide(a, idx, v))
            s.pack(side="left", fill="x", expand=True)
            e = ttk.Entry(row, textvariable=vars[i], width=5)
            e.pack(side="left", padx=2)
            e.bind("<Return>", lambda e, a=arm_id, idx=i: self.entry_submit(a, idx))
            ttk.Checkbutton(row, text="Inv", variable=invs[i]).pack(side="left")

    # ================= SCENE & FILES =================
    def refresh_scenes(self):
        scenes = [d for d in os.listdir(BASE_REC_DIR) if os.path.isdir(os.path.join(BASE_REC_DIR, d))]
        self.combo_scenes['values'] = scenes
        if scenes and not self.current_scene.get():
            self.combo_scenes.current(0)
            self.on_scene_selected(None)

    def create_scene(self):
        name = simpledialog.askstring("New Scene", "Scene Name (e.g. Making_Crepes):")
        if name:
            path = os.path.join(BASE_REC_DIR, name)
            if not os.path.exists(path):
                os.makedirs(path)
                self.refresh_scenes()
                self.combo_scenes.set(name)
                self.on_scene_selected(None)

    def on_scene_selected(self, event):
        scene = self.current_scene.get()
        self.list_files.delete(0, tk.END)
        path = os.path.join(BASE_REC_DIR, scene)
        if not os.path.exists(path): return

        files = sorted([f for f in os.listdir(path) if f.endswith(".json")])
        for f in files:
            self.list_files.insert(tk.END, f)

    def get_next_filename(self):
        scene = self.current_scene.get()
        if not scene: return None
        
        path = os.path.join(BASE_REC_DIR, scene)
        if not os.path.exists(path):
            try:
                os.makedirs(path)
            except:
                return None

        files = []
        try:
            files = [f for f in os.listdir(path) if f.endswith(".json")]
        except Exception as e:
            print(f"Error listing dir: {e}")
        
        # Determine next index
        idx = 1
        if files:
            try:
                # Try to find max int prefix "01_"
                prefixes = [int(f.split('_')[0]) for f in files if f[0].isdigit()]
                if prefixes:
                    idx = max(prefixes) + 1
            except: pass
            
        base = self.step_name.get().strip() or "Step"
        # Sanitize
        base = "".join(c for c in base if c.isalnum() or c in (' ', '_')).replace(' ', '_')
        return f"{idx:02d}_{base}.json"

    def on_file_select(self, event):
        sel = self.list_files.curselection()
        if not sel: return
        fname = self.list_files.get(sel[0])
        scene = self.current_scene.get()
        path = os.path.join(BASE_REC_DIR, scene, fname)
        
        try:
            with open(path, 'r') as f:
                data = json.load(f)
            
            meta = data.get("meta", {})
            dur = meta.get("duration_sec", "?")
            
            info = f"File: {fname}\n"
            info += f"Dur: {dur}s\n"
            info += f"Points: {len(data.get('timeline', []))}\n"
            info += "-"*20 + "\n"
            
            # Start Pos
            s1 = data.get("start_arm1", [])
            s2 = data.get("start_arm2", [])
            info += f"Start A1: {s1}\n"
            info += f"Start A2: {s2}\n"
            
            # End Pos
            e1 = data.get("end_arm1", [])
            e2 = data.get("end_arm2", [])
            info += f"End A1: {e1}\n"
            info += f"End A2: {e2}\n"
            
            self.txt_props.config(state="normal")
            self.txt_props.delete("1.0", tk.END)
            self.txt_props.insert(tk.END, info)
            self.txt_props.config(state="disabled")
            
        except Exception as e:
            pass

    def add_to_playlist(self):
        sel = self.list_files.curselection()
        if not sel: return
        fname = self.list_files.get(sel[0])
        scene = self.current_scene.get()
        
        full_path = os.path.join(BASE_REC_DIR, scene, fname)
        display_name = f"[{scene}] {fname}"
        
        self.playlist.append(full_path)
        self.list_playlist.insert(tk.END, display_name)

    def rem_from_playlist(self):
        sel = self.list_playlist.curselection()
        if not sel: return
        idx = sel[0]
        self.playlist.pop(idx)
        self.list_playlist.delete(idx)

    # ================= LOGIC: RECORDING =================
    def toggle_rec(self):
        if not self.current_scene.get():
            messagebox.showerror("Error", "Select a scene first!")
            return

        if not self.is_recording:
            # START
            self.is_recording = True
            self.rec_start_time = time.time()
            self.rec_data = []
            self.btn_rec.config(text="STOP REC")
            self.lbl_rec.config(text="🔴 REC", foreground="red")
            
            # Snapshot start
            self.rec_snapshot(force=True)
            print("Recording started.")
        else:
            # STOP & SAVE
            self.is_recording = False
            self.btn_rec.config(text="REC")
            self.lbl_rec.config(text="⚫", foreground="gray")
            
            print(f"Stopping recording. Captured {len(self.rec_data)} points.")
            
            if len(self.rec_data) == 0:
                messagebox.showwarning("Warning", "No data recorded! (Length is 0)")
                return
                
            self.save_sequence()

    def rec_snapshot(self, force=False):
        if not self.is_recording: return
        
        try:
            t = round(time.time() - self.rec_start_time, 3)
            
            # Get current states
            v1 = [int(v) for v in self.arm1_values]
            v2 = [int(v) for v in self.arm2_values]
            m1 = [bool(self.arm1_magnets[0].get()), bool(self.arm1_magnets[1].get())]
            
            # Check against last point
            if not force and self.rec_data:
                last = self.rec_data[-1]
                if (last["a1"] == v1 and last["a2"] == v2 and last["m1"] == m1):
                    return
                    
            self.rec_data.append({
                "t": t,
                "a1": v1,
                "a2": v2,
                "m1": m1
            })
        except Exception as e:
            print(f"Snapshot error: {e}")
            # If we repeatedly fail, we might want to know
            if len(self.rec_data) == 0 and force:
                 messagebox.showerror("Snapshot Error", f"Failed to record initial point:\n{e}")

    def save_sequence(self):
        try:
            if not self.rec_data: return
            
            scene = self.current_scene.get()
            fname = self.get_next_filename()
            if not fname: 
                messagebox.showerror("Error", "Could not generate filename. Check path permissions.")
                return
            
            path = os.path.join(BASE_REC_DIR, scene, fname)
            
            # Meta
            start_node = self.rec_data[0]
            end_node = self.rec_data[-1]
            
            payload = {
                "meta": {
                    "scene": scene,
                    "name": fname,
                    "created": time.ctime(),
                    "duration_sec": end_node["t"]
                },
                "start_arm1": start_node["a1"],
                "start_arm2": start_node["a2"],
                "end_arm1": end_node["a1"],
                "end_arm2": end_node["a2"],
                "timeline": self.rec_data
            }
            
            with open(path, 'w') as f:
                json.dump(payload, f, indent=2)
                
            messagebox.showinfo("Saved", f"Sequence stored as:\n{fname}")
            self.on_scene_selected(None) # Refresh list
            
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    # ================= LOGIC: PLAYBACK =================
    def play_playlist(self):
        if not self.playlist:
            messagebox.showwarning("Empty Playlist", "Please add sequences to the playlist first.")
            return
        
        # UI Feedback
        self.btn_rec.config(state="disabled") # Lock recording
        self.stop_event.clear()
        
        threading.Thread(target=self.run_playlist, daemon=True).start()

    def stop_playback(self):
        self.stop_event.set()

    def run_playlist(self): # Threaded
        self.is_playing = True
        # Visual indicator
        self.root.after(0, lambda: self.root.title("Dual Arm Director's Console - ▶ PLAYING..."))
        
        try:
            for fpath in self.playlist:
                if self.stop_event.is_set(): break
                
                # Check file existence
                if not os.path.exists(fpath):
                    print(f"File not found: {fpath}")
                    continue

                try:
                    with open(fpath, 'r') as f:
                        data = json.load(f)
                    
                    timeline = data.get("timeline", [])
                    if not timeline: continue
                    
                    print(f"Playing {os.path.basename(fpath)} ({len(timeline)} points)")

                    # Reset base time for this sequence
                    start_real_time = time.time()
                    
                    for point in timeline:
                        if self.stop_event.is_set(): break
                        
                        target_time = point.get("t", 0)
                        
                        # Wait loop
                        while (time.time() - start_real_time) < target_time:
                            if self.stop_event.is_set(): break
                            time.sleep(0.01)
                            
                        # Apply
                        self.apply_state(point)
                        
                except Exception as e:
                    err = f"Error playing {os.path.basename(fpath)}: {e}"
                    print(err)
                    self.root.after(0, lambda m=err: messagebox.showerror("Playback Error", m))
                    break # Stop playlist on error
                    
        finally:
            self.is_playing = False
            self.root.after(0, lambda: self.root.title("Dual Arm Director's Console"))
            self.root.after(0, lambda: self.btn_rec.config(state="normal"))

    def apply_state(self, point):
        # Update Data - Robust usage with defaults
        if "a1" in point: self.arm1_values = point["a1"]
        if "a2" in point: self.arm2_values = point["a2"]
        
        # Update Magnets (Check change)
        if "m1" in point:
            mag_state = point["m1"]
            if mag_state != self.arm1_last_magnets:
                self.send_magnet_cmd(1, 0, mag_state[0])
                self.send_magnet_cmd(1, 1, mag_state[1])
                self.arm1_last_magnets = mag_state
            
        # Update Sliders (Visual) - Schedule on Main Thread
        self.root.after(0, lambda p=point: self.sync_ui_to_state(p))

    def sync_ui_to_state(self, point):
        try:
            if "a1" in point:
                for i in range(5):
                    self.arm1_sliders[i].set(point["a1"][i])
                    self.arm1_vars[i].set(point["a1"][i])
            if "a2" in point:
                for i in range(5):
                    self.arm2_sliders[i].set(point["a2"][i])
                    self.arm2_vars[i].set(point["a2"][i])
            if "m1" in point:
                self.arm1_magnets[0].set(point["m1"][0])
                self.arm1_magnets[1].set(point["m1"][1])
        except: pass

    # ================= LOW LEVEL =================
    def on_slide(self, arm_id, idx, val):
        arr = self.arm1_values if arm_id == 1 else self.arm2_values
        arr[idx] = int(float(val))
        
        v_arr = self.arm1_vars if arm_id == 1 else self.arm2_vars
        v_arr[idx].set(str(arr[idx]))
        
        if self.is_recording: self.rec_snapshot()

    def entry_submit(self, arm_id, idx):
        v_arr = self.arm1_vars if arm_id == 1 else self.arm2_vars
        s_arr = self.arm1_sliders if arm_id == 1 else self.arm2_sliders
        arr = self.arm1_values if arm_id == 1 else self.arm2_values
        
        try:
            val = int(v_arr[idx].get())
            val = max(PULSE_MIN, min(PULSE_MAX, val))
            arr[idx] = val
            s_arr[idx].set(val)
            if self.is_recording: self.rec_snapshot()
        except: pass
        self.root.focus()

    def update_magnets(self, from_ui=False):
        # Triggered by checkbox
        m1 = self.arm1_magnets[0].get()
        m2 = self.arm1_magnets[1].get()
        
        self.send_magnet_cmd(1, 0, m1)
        self.send_magnet_cmd(1, 1, m2)
        self.arm1_last_magnets = [m1, m2]
        
        if self.is_recording: self.rec_snapshot()

    # --- Serial & Keys (Refactored for efficiency) ---
    def control_loop(self):
        if self.active_keys:
            step = self.speed.get()
            changed = False
            
            for k in list(self.active_keys):
                # ARM 1
                if k in KEY_MAP_ARM1:
                    i, d = KEY_MAP_ARM1[k]
                    if self.arm1_inversions[i].get(): d *= -1
                    new_v = self.arm1_values[i] + (d * step)
                    new_v = max(PULSE_MIN, min(PULSE_MAX, new_v))
                    if new_v != self.arm1_values[i]:
                        self.arm1_values[i] = int(new_v)
                        self.arm1_sliders[i].set(new_v)
                        self.arm1_vars[i].set(int(new_v))
                        changed = True
                
                # ARM 2
                if k in KEY_MAP_ARM2:
                    i, d = KEY_MAP_ARM2[k]
                    if self.arm2_inversions[i].get(): d *= -1
                    new_v = self.arm2_values[i] + (d * step)
                    new_v = max(PULSE_MIN, min(PULSE_MAX, new_v))
                    if new_v != self.arm2_values[i]:
                        self.arm2_values[i] = int(new_v)
                        self.arm2_sliders[i].set(new_v)
                        self.arm2_vars[i].set(int(new_v))
                        changed = True
            
            if changed and self.is_recording:
                self.rec_snapshot()
        
        self.root.after(30, self.control_loop)

    def serial_loop(self):
        # Send Arm 1
        if self.is_connected1 and self.serial_port1:
            for i in range(5):
                v = int(self.arm1_values[i])
                if v != self.arm1_last_sent[i]:
                    try:
                        self.serial_port1.write(f"s{i+1}-{v}\n".encode())
                        self.arm1_last_sent[i] = v
                    except: pass
        
        # Send Arm 2
        if self.is_connected2 and self.serial_port2:
            for i in range(5):
                v = int(self.arm2_values[i])
                if v != self.arm2_last_sent[i]:
                    try:
                        self.serial_port2.write(f"s{i+1}-{v}\n".encode())
                        self.arm2_last_sent[i] = v
                    except: pass
        
        self.root.after(50, self.serial_loop)

    def send_magnet_cmd(self, arm, magnet_idx, state):
        # arm is 1 or 2. magnet_idx is 0 or 1. state is bool.
        # Only Arm 1 has magnets currently
        if arm == 1 and self.is_connected1 and self.serial_port1:
            cmd = f"m{magnet_idx+1}-{1 if state else 0}\n"
            try:
                self.serial_port1.write(cmd.encode())
            except: pass

    def on_key_press(self, e):
        if not isinstance(e.widget, ttk.Entry):
            self.active_keys.add(e.char.lower())

    def on_key_release(self, e):
        if not isinstance(e.widget, ttk.Entry):
            k = e.char.lower()
            if k in self.active_keys:
                self.active_keys.discard(k)
                if self.is_recording: self.rec_snapshot()

    def get_ports(self):
        return [p.device for p in serial.tools.list_ports.comports()]

    def toggle_conn(self, idx, cb):
        port = cb.get()
        if not port: return
        
        try:
            ser =  serial.Serial(port, BAUD_RATE, timeout=0.1)
            if idx == 1:
                self.serial_port1 = ser
                self.is_connected1 = True
                self.btn_p1.config(text="Disconnect", state="disabled")
            else:
                self.serial_port2 = ser
                self.is_connected2 = True
                self.btn_p2.config(text="Disconnect", state="disabled")
        except Exception as e:
            messagebox.showerror("Conn Error", str(e))

    def reset_all(self):
        for i in range(5):
            self.arm1_values[i] = NEUTRAL
            self.arm2_values[i] = NEUTRAL
            self.arm1_sliders[i].set(NEUTRAL)
            self.arm2_sliders[i].set(NEUTRAL)
            self.arm1_vars[i].set(str(NEUTRAL))
            self.arm2_vars[i].set(str(NEUTRAL))
        if self.is_recording: self.rec_snapshot()


if __name__ == "__main__":
    root = tk.Tk()
    app = DualArmManualControl(root)
    root.mainloop()
