import tkinter as tk
from tkinter import ttk, scrolledtext
import serial
import serial.tools.list_ports
import time
import threading

# ==========================================
# CONFIGURATION
# ==========================================
BAUD_RATE = 115200

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
        self.root.geometry("600x700")

        self.serial_port = None
        self.is_connected = False
        
        # State Variables
        self.servo_values = [1500] * 5 # Current pulse width
        self.servo_vars = [tk.StringVar(value="1500") for _ in range(5)]
        self.last_sent_values = [1500] * 5
        self.inversions = [tk.BooleanVar(value=False) for _ in range(5)] # True = Inverted
        
        # Movement State
        self.active_keys = set() # Keys currently pressed
        self.speed = tk.IntVar(value=5) # Step size per tick (higher = faster)
        
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
            
            # Name & Keys
            ttk.Label(f, text=f"{keys_info[i][0]} [{keys_info[i][1]}]", width=25, anchor="w").pack(side="left", padx=5)
            
            # Value Display
            lbl_val = ttk.Label(f, textvariable=self.servo_vars[i], width=8, font=("Consolas", 10, "bold"))
            lbl_val.pack(side="left", padx=5)
            
            # Invert Checkbox
            cb = ttk.Checkbutton(f, text="Invert", variable=self.inversions[i])
            cb.pack(side="right", padx=10)

        # --- Log Frame ---
        log_frame = ttk.LabelFrame(self.root, text="Logs")
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, state='disabled', height=10)
        self.log_text.pack(fill="both", expand=True, padx=5, pady=5)
        
        # Instructions
        instr_lbl = ttk.Label(self.root, text="Hold keys to move servos. Ensure caps lock is OFF.", foreground="gray")
        instr_lbl.pack(pady=5)

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

    def log_message(self, msg):
        self.log_text.config(state='normal')
        self.log_text.insert(tk.END, f"> {msg}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state='disabled')

    def on_key_press(self, event):
        char = event.char.lower()
        if char in KEY_MAP:
            self.active_keys.add(char)

    def on_key_release(self, event):
        char = event.char.lower()
        if char in KEY_MAP:
            self.active_keys.discard(char)

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
