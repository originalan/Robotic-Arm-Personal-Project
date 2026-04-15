import tkinter as tk
import csv
from tkinter import filedialog
from tkinter import ttk
import ikpy.chain
import numpy as np
import serial
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.ticker as ticker

# --- CONFIGURATION ---
URDF_FILE = "simple_arm.urdf"
PORT = 'COM6' 
BAUD = 115200
INCH_TO_METERS = 0.0254
TOLERANCE_INCHES = 0.5 
GRIPPER_OPEN = 1
GRIPPER_CLOSED = 0
STEP_SIZE = 0.25
PLAYBACK_DELAY_MS = 1000
ANIMATION_DELAY_MS = 1

class RobotArmApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotic Arm Control Studio")
        self.root.geometry("1200x750")
        
        style = ttk.Style()
        if "clam" in style.theme_names():
            style.theme_use("clam")
        
        style.configure("Header.TLabel", font=("Segoe UI", 12, "bold"))
        style.configure("Status.TLabel", font=("Consolas", 11), foreground="#005500")
        style.configure("Estop.TLabel", font=("Consolas", 11, "bold"), foreground="#aa0000")

        # --- STATE VARIABLES ---
        self.curr_x, self.curr_y, self.curr_z = 5.0, 0.0, 5.0
        self.curr_w = 90  
        self.curr_g = GRIPPER_CLOSED
        self.my_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE)
        
        self.waypoints = []
        self.is_playing = False
        self.is_moving = False 
        self.e_stop_active = False # NEW: Global lock for the robot
        
        # --- INITIALIZE SERIAL ---
        self.ser = None
        self.connect_serial()

        # --- BUILD UI ---
        self.build_ui()
        
        # --- INITIAL DRAW ---
        self.move_arm(self.curr_x, self.curr_y, self.curr_z, self.curr_w, self.curr_g)

        # --- BIND KEYBOARD ---
        self.root.bind('<KeyPress>', self.on_key_press)
        self.root.bind('<space>', self.trigger_estop) # Slam spacebar to E-Stop

        # --- BIND WINDOW CLOSE ---
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def connect_serial(self):
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
            print("Waking up Arduino...")
            
            # CRITICAL: Force Python to wait 2 seconds for the Arduino to finish rebooting
            time.sleep(2) 
            
            # Flush out any random junk data that happened during boot
            self.ser.reset_input_buffer() 
            self.ser.reset_output_buffer()

            print("Waiting for Arduino handshake...")
            # We add a safety counter so it doesn't hang the GUI forever if it fails
            attempts = 0
            while attempts < 50: 
                if self.ser.in_waiting > 0:
                    msg = self.ser.readline().decode('utf-8').strip()
                    if msg == "READY":
                        print("Arduino Handshake Complete!")
                        return
                time.sleep(0.1)
                attempts += 1
                
            print("Warning: Handshake timed out. Trying to proceed anyway.")
            
        except Exception as e:
            print(f"Serial Error: {e}. Running in Simulation Mode.")
            self.ser = None

    def build_ui(self):
        left_col = ttk.Frame(self.root, padding=10)
        left_col.pack(side=tk.LEFT, fill=tk.Y)
        mid_col = ttk.Frame(self.root, padding=10)
        mid_col.pack(side=tk.LEFT, fill=tk.Y)
        right_col = ttk.Frame(self.root, padding=10)
        right_col.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # --- LEFT COLUMN ---
        
        # E-STOP BOX (Uses standard tk.Button for easy red background)
        estop_frame = tk.Frame(left_col)
        estop_frame.pack(fill=tk.X, pady=(0, 10))
        self.btn_estop = tk.Button(estop_frame, text="EMERGENCY STOP (SPACE)", bg="#dd0000", fg="white", 
                                   font=("Segoe UI", 12, "bold"), height=2, command=self.trigger_estop)
        self.btn_estop.pack(fill=tk.X)
        self.btn_reset = ttk.Button(estop_frame, text="Reset Safety Lockout", command=self.reset_estop, state=tk.DISABLED)
        self.btn_reset.pack(fill=tk.X, pady=2)

        # Status Box
        status_frame = ttk.LabelFrame(left_col, text="System Status", padding=10)
        status_frame.pack(fill=tk.X, pady=(0, 10))
        status_container = ttk.Frame(status_frame, width=300, height=50) # 50px fits exactly 2-3 lines of text
        status_container.pack_propagate(False) # This is the magic lock! Stops it from stretching.
        status_container.pack()

        self.status_var = tk.StringVar(value="System Ready")
        
        self.status_label = ttk.Label(
            status_container,         # Put the label inside the rigid container
            textvariable=self.status_var, 
            style="Status.TLabel",
            anchor=tk.CENTER,     
            justify=tk.CENTER,    
            wraplength=290            # Wrap just before it hits the 300px wall
        )
        # expand=True keeps the text perfectly centered vertically and horizontally inside the 50px box
        self.status_label.pack(expand=True, fill=tk.BOTH)

        # Direct Input Box
        input_frame = ttk.LabelFrame(left_col, text="Direct Coordinates (inches)", padding=10)
        input_frame.pack(fill=tk.X, pady=5)
        ttk.Label(input_frame, text="X:").grid(row=0, column=0, padx=2)
        self.entry_x = ttk.Entry(input_frame, width=6); self.entry_x.grid(row=0, column=1, padx=2)
        ttk.Label(input_frame, text="Y:").grid(row=0, column=2, padx=2)
        self.entry_y = ttk.Entry(input_frame, width=6); self.entry_y.grid(row=0, column=3, padx=2)
        ttk.Label(input_frame, text="Z:").grid(row=0, column=4, padx=2)
        self.entry_z = ttk.Entry(input_frame, width=6); self.entry_z.grid(row=0, column=5, padx=2)
        ttk.Button(input_frame, text="Move to Target", command=self.go_to_direct_input).grid(row=1, column=0, columnspan=6, pady=10, sticky="ew")

        # Jog Box
        jog_frame = ttk.LabelFrame(left_col, text="Jog Control", padding=10)
        jog_frame.pack(fill=tk.X, pady=5)
        ttk.Button(jog_frame, text="+X (W)", width=8, command=lambda: self.jog(STEP_SIZE, 0, 0)).grid(row=0, column=1, pady=2)
        ttk.Button(jog_frame, text="-Y (A)", width=8, command=lambda: self.jog(0, -STEP_SIZE, 0)).grid(row=1, column=0, padx=2)
        ttk.Button(jog_frame, text="-X (S)", width=8, command=lambda: self.jog(-STEP_SIZE, 0, 0)).grid(row=1, column=1, padx=2)
        ttk.Button(jog_frame, text="+Y (D)", width=8, command=lambda: self.jog(0, STEP_SIZE, 0)).grid(row=1, column=2, padx=2)
        ttk.Button(jog_frame, text="+Z (Up)", width=8, command=lambda: self.jog(0, 0, STEP_SIZE)).grid(row=0, column=3, padx=15, pady=2)
        ttk.Button(jog_frame, text="-Z (Dn)", width=8, command=lambda: self.jog(0, 0, -STEP_SIZE)).grid(row=1, column=3, padx=15)

        # Wrist Box
        wrist_frame = ttk.LabelFrame(left_col, text="Wrist Rotation (Q / E)", padding=10)
        wrist_frame.pack(fill=tk.X, pady=5)
        self.wrist_slider = ttk.Scale(wrist_frame, from_=0, to=180, orient=tk.HORIZONTAL)
        self.wrist_label = ttk.Label(wrist_frame, text=f"Pending Angle: {self.curr_w}°")
        self.wrist_slider.set(self.curr_w)
        self.wrist_slider.config(command=self.on_wrist_slide)
        self.wrist_slider.pack(fill=tk.X, padx=5)
        self.wrist_label.pack(pady=2)
        ttk.Button(wrist_frame, text="APPLY WRIST ANGLE", command=self.apply_wrist_angle).pack(pady=5, fill=tk.X)

        # Gripper Box
        grip_frame = ttk.LabelFrame(left_col, text="Gripper", padding=10)
        grip_frame.pack(fill=tk.X, pady=5)
        ttk.Button(grip_frame, text="OPEN (O)", command=lambda: self.toggle_gripper(GRIPPER_OPEN)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(grip_frame, text="CLOSE (C)", command=lambda: self.toggle_gripper(GRIPPER_CLOSED)).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # --- MIDDLE COLUMN ---
        wp_frame = ttk.LabelFrame(mid_col, text="Sequence Recorder", padding=10)
        wp_frame.pack(fill=tk.BOTH, expand=True)
        self.wp_listbox = tk.Listbox(wp_frame, width=35, height=20, font=("Consolas", 10))
        self.wp_listbox.pack(pady=5, fill=tk.BOTH, expand=True)
        btn_frame = ttk.Frame(wp_frame)
        btn_frame.pack(fill=tk.X)
        ttk.Button(btn_frame, text="Save Current Pose", command=self.save_waypoint).pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="Clear List", command=self.clear_waypoints).pack(fill=tk.X, pady=2)
        ttk.Button(btn_frame, text="▶ PLAY SEQUENCE", command=self.play_waypoints).pack(fill=tk.X, pady=10)

        csv_frame = ttk.Frame(wp_frame)
        csv_frame.pack(fill=tk.X, pady=(10, 0))
        ttk.Button(csv_frame, text="Load CSV", command=self.load_waypoints_csv).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        ttk.Button(csv_frame, text="Save CSV", command=self.save_waypoints_csv).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # --- RIGHT COLUMN ---
        plot_frame = ttk.LabelFrame(right_col, text="Live 3D Visualization", padding=5)
        plot_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(6, 6), dpi=130, facecolor='#f0f0f0') 
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Force the 3D plot area itself to be completely transparent
        self.ax.set_facecolor('none') 
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def on_closing(self):
        """Fires right before the window closes to relax the robot arm."""
        print("Shutting down... Relaxing servos.")
        try:
            if self.ser and self.ser.is_open:
                # 1. Clear out any half-sent commands in Python's queue
                self.ser.reset_output_buffer()
                
                # 2. Add a newline at the start to clear any half-read junk in the Arduino's memory
                # then send the actual kill signal
                kill_cmd = b"\n-1,-1,-1,-1,-1\n"
                self.ser.write(kill_cmd)
                
                # 3. FORCE the computer to push the data out of the USB port immediately
                self.ser.flush() 
                
                # 4. Give the Arduino enough time to read it before cutting the cord
                time.sleep(0.2) 
                
                self.ser.close()
                print("Serial port closed safely.")
        except Exception as e:
            print(f"Serial shutdown error: {e}")
        finally:
            # 5. Safely close the UI without triggering the TclError crash
            try:
                # Only attempt to destroy the window if it still physically exists
                if self.root.winfo_exists():
                    self.root.destroy()
            except Exception:
                pass # The window is already dead, which is exactly what we want!

    # --- SAFETY FUNCTIONS ---
    def trigger_estop(self, event=None):
        self.e_stop_active = True
        self.is_playing = False
        self.is_moving = False
        self.status_var.set("E-STOP TRIGGERED! MOTORS HALTED.")
        self.status_label.config(style="Estop.TLabel")
        self.btn_estop.config(state=tk.DISABLED, bg="#550000")
        self.btn_reset.config(state=tk.NORMAL)
        # Visually clear sequence selection to show playback stopped
        self.wp_listbox.selection_clear(0, tk.END)

    def reset_estop(self):
        self.e_stop_active = False
        self.status_var.set("System Ready")
        self.status_label.config(style="Status.TLabel")
        self.btn_estop.config(state=tk.NORMAL, bg="#dd0000")
        self.btn_reset.config(state=tk.DISABLED)

    # --- LOGIC FUNCTIONS ---
    def go_to_direct_input(self):
        if self.is_playing or self.is_moving or self.e_stop_active: return
        try:
            x, y, z = float(self.entry_x.get()), float(self.entry_y.get()), float(self.entry_z.get())
            self.move_arm_smooth(x, y, z, self.curr_w, self.curr_g)
        except ValueError:
            self.status_var.set("Error: Invalid Number!")

    def jog(self, dx, dy, dz):
        if self.is_playing or self.is_moving or self.e_stop_active: return
        self.move_arm(self.curr_x + dx, self.curr_y + dy, self.curr_z + dz, self.curr_w, self.curr_g)

    def jog_wrist(self, delta_w):
        if self.is_playing or self.is_moving or self.e_stop_active: return
        new_w = max(0, min(180, self.curr_w + delta_w))
        self.wrist_slider.set(new_w) 
        self.move_arm(self.curr_x, self.curr_y, self.curr_z, new_w, self.curr_g)

    def on_wrist_slide(self, val):
        if self.is_playing: return
        angle = int(float(val))
        self.wrist_label.config(text=f"Pending Angle: {angle}°")

    def apply_wrist_angle(self):
        if self.is_playing or self.is_moving or self.e_stop_active: return
        angle = int(float(self.wrist_slider.get()))
        self.move_arm_smooth(self.curr_x, self.curr_y, self.curr_z, angle, self.curr_g)

    def toggle_gripper(self, state):
        if self.is_playing or self.is_moving or self.e_stop_active: return
        self.move_arm(self.curr_x, self.curr_y, self.curr_z, self.curr_w, state)

    def on_key_press(self, event):
        # Allow spacebar through, but block everything else during E-Stop/movement
        if event.keysym == 'space': return 
        if self.is_playing or self.is_moving or self.e_stop_active: return
        
        char = event.char.lower()
        if char == 'w': self.jog(STEP_SIZE, 0, 0)
        elif char == 's': self.jog(-STEP_SIZE, 0, 0)
        elif char == 'a': self.jog(0, -STEP_SIZE, 0)
        elif char == 'd': self.jog(0, STEP_SIZE, 0)
        elif char == 'q': self.jog_wrist(-5)
        elif char == 'e': self.jog_wrist(5) 
        elif char == 'o': self.toggle_gripper(GRIPPER_OPEN)
        elif char == 'c': self.toggle_gripper(GRIPPER_CLOSED)
        elif event.keysym == 'Up': self.jog(0, 0, STEP_SIZE)
        elif event.keysym == 'Down': self.jog(0, 0, -STEP_SIZE)

    # --- WAYPOINT FUNCTIONS ---
    def save_waypoint(self):
        g_text = "OPEN" if self.curr_g == GRIPPER_OPEN else "CLOSED"
        self.waypoints.append((self.curr_x, self.curr_y, self.curr_z, self.curr_w, self.curr_g))
        self.wp_listbox.insert(tk.END, f"Pt {len(self.waypoints)}: ({self.curr_x:.1f}, {self.curr_y:.1f}, {self.curr_z:.1f}) | W:{self.curr_w}° | {g_text}")

    def save_waypoints_csv(self):
        """Opens a prompt to save the current waypoint sequence to a CSV file."""
        if self.is_playing or self.is_moving or self.e_stop_active:
            self.status_var.set("Cannot save CSV while moving!")
            return

        if not self.waypoints:
            self.status_var.set("No waypoints to save!")
            return
            
        # Open standard OS Save File dialog
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
            title="Save Waypoints"
        )
        
        if not filepath:
            return # User clicked cancel
            
        try:
            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Write a header row so the CSV is easy to read in Excel
                writer.writerow(["X", "Y", "Z", "Wrist_Angle", "Gripper_State"]) 
                
                # Write the actual data tuples
                for wp in self.waypoints:
                    writer.writerow(wp)
                    
            self.status_var.set(f"Saved {len(self.waypoints)} waypoints to CSV.")
        except Exception as e:
            self.status_var.set("Error saving CSV!")
            print(f"CSV Save Error: {e}")

    def load_waypoints_csv(self):
        """Opens a prompt to load a waypoint sequence from a CSV file."""
        if self.is_playing or self.is_moving or self.e_stop_active:
            self.status_var.set("Cannot load CSV while moving/stopped.")
            return
            
        # Open standard OS Open File dialog
        filepath = filedialog.askopenfilename(
            filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")],
            title="Load Waypoints"
        )
        
        if not filepath:
            return # User clicked cancel
            
        try:
            with open(filepath, mode='r') as file:
                reader = csv.reader(file)
                
                # Check the first row. If it's the header we made, skip it.
                first_row = next(reader, None)
                if first_row and not first_row[0].replace('.','',1).isdigit() and not first_row[0].lstrip('-').replace('.','',1).isdigit():
                    pass # It is text (header), do nothing
                else:
                    # It's actual data, rewind the file so we don't lose the first waypoint
                    file.seek(0)
                    reader = csv.reader(file)
                    
                # Clear the current UI list and memory list safely
                self.clear_waypoints() 
                
                # Parse the CSV and reconstruct the list
                for row in reader:
                    if len(row) >= 5:
                        x, y, z = float(row[0]), float(row[1]), float(row[2])
                        w = float(row[3])
                        g = int(float(row[4]))
                        
                        self.waypoints.append((x, y, z, w, g))
                        
                        # Rebuild the Listbox UI text
                        g_text = "OPEN" if g == GRIPPER_OPEN else "CLOSED"
                        self.wp_listbox.insert(tk.END, f"Pt {len(self.waypoints)}: ({x:.1f}, {y:.1f}, {z:.1f}) | W:{w}° | {g_text}")
                        
            self.status_var.set(f"Loaded {len(self.waypoints)} waypoints from CSV.")
        except Exception as e:
            self.status_var.set("Error loading CSV! Is the format correct?")
            print(f"CSV Load Error: {e}")

    def clear_waypoints(self):
        self.waypoints.clear()
        self.wp_listbox.delete(0, tk.END)
        if not self.e_stop_active:
            self.status_var.set("Waypoints Cleared.")

    def play_waypoints(self):
        if not self.waypoints or self.is_playing or self.e_stop_active: return
        self.is_playing = True
        self.current_wp_index = 0
        self.status_var.set("Playing Sequence...")
        self.execute_next_waypoint()

    def execute_next_waypoint(self):
        if self.e_stop_active: return # Abort playback completely
        
        if self.current_wp_index < len(self.waypoints):
            self.wp_listbox.selection_clear(0, tk.END)
            self.wp_listbox.selection_set(self.current_wp_index)
            
            x, y, z, w, g = self.waypoints[self.current_wp_index]
            self.wrist_slider.set(w)
            self.wrist_label.config(text=f"Angle: {w}°")
            
            self.move_arm_smooth(x, y, z, w, g, callback=self.on_waypoint_reached)
        else:
            self.is_playing = False
            self.status_var.set("Sequence Complete!")
            self.wp_listbox.selection_clear(0, tk.END)

    def on_waypoint_reached(self):
        if self.e_stop_active: return # Don't schedule next point if estop was hit
        self.current_wp_index += 1
        self.root.after(PLAYBACK_DELAY_MS, self.execute_next_waypoint)

    # --- TRAJECTORY GENERATION (SMOOTHING) ---
    def move_arm_smooth(self, t_x, t_y, t_z, t_w, t_g, callback=None):
        if self.e_stop_active:
            return

        # --- 1. PRE-FLIGHT REACHABILITY CHECK ---
        # Test the final destination before making any moves
        target_m = [t_x * INCH_TO_METERS, t_y * INCH_TO_METERS, t_z * INCH_TO_METERS]
        joint_angles = self.my_chain.inverse_kinematics(target_m)
        computed_pos_m = self.my_chain.forward_kinematics(joint_angles)[:3, 3]
        error_in = np.linalg.norm(computed_pos_m - target_m) / INCH_TO_METERS
        
        if error_in > TOLERANCE_INCHES:
            self.status_var.set(f"ABORTED: Target is UNREACHABLE! (Error: {error_in:.2f} in)")
            
            # Keep the arm exactly where it is, but draw the unreachable target in the 3D plot
            current_m = [self.curr_x * INCH_TO_METERS, self.curr_y * INCH_TO_METERS, self.curr_z * INCH_TO_METERS]
            current_angles = self.my_chain.inverse_kinematics(current_m)
            self.update_plot(current_angles, target_m, False)
            
            # If this happened during sequence playback, cancel the playback
            self.is_playing = False
            self.wp_listbox.selection_clear(0, tk.END)
            return

        # --- 2. PROCEED WITH TRAJECTORY GENERATION ---
        self.is_moving = True
        dist = np.sqrt((t_x - self.curr_x)**2 + (t_y - self.curr_y)**2 + (t_z - self.curr_z)**2)
        steps = max(10, int(dist * 15))
        w_dist = abs(t_w - self.curr_w)
        
        if dist == 0 and w_dist > 0:
            steps = max(10, int(w_dist / 2))

        # --- COSINE INTERPOLATION ---
        # 1. Create a linear time array from 0.0 to 1.0
        t = np.linspace(0, 1, steps)
        
        # 2. Warp the linear time into a smooth S-curve using cosine
        smooth_t = (1 - np.cos(t * np.pi)) / 2 
        
        # 3. Apply the S-curve to the start and end coordinates
        xs = self.curr_x + (t_x - self.curr_x) * smooth_t
        ys = self.curr_y + (t_y - self.curr_y) * smooth_t
        zs = self.curr_z + (t_z - self.curr_z) * smooth_t
        ws = self.curr_w + (t_w - self.curr_w) * smooth_t
        
        self.status_var.set(f"Gliding to Target ({steps} frames)...")
        self._animate_step(xs, ys, zs, ws, t_g, steps, 0, callback)

    def _animate_step(self, xs, ys, zs, ws, target_g, total_steps, step, callback):
        # CRITICAL E-STOP CHECK: Abort the animation loop immediately
        if self.e_stop_active: 
            self.is_moving = False
            return 
            
        if step < total_steps:
            is_last = (step == total_steps - 1)
            self.move_arm(xs[step], ys[step], zs[step], ws[step], target_g, update_visuals=is_last)
            self.root.after(ANIMATION_DELAY_MS, self._animate_step, xs, ys, zs, ws, target_g, total_steps, step + 1, callback)
        else:
            self.is_moving = False
            if callback:
                callback()

    # --- CORE MOVEMENT ---
    def move_arm(self, target_x, target_y, target_z, target_w, target_g, update_visuals=True):
        if self.e_stop_active: return # Hardware lock
        
        target_m = [target_x * INCH_TO_METERS, target_y * INCH_TO_METERS, target_z * INCH_TO_METERS]
        joint_angles = self.my_chain.inverse_kinematics(target_m)
        
        computed_pos_m = self.my_chain.forward_kinematics(joint_angles)[:3, 3]
        error_in = np.linalg.norm(computed_pos_m - target_m) / INCH_TO_METERS
        reachable = error_in <= TOLERANCE_INCHES

        if reachable:
            self.curr_x, self.curr_y, self.curr_z, self.curr_w, self.curr_g = target_x, target_y, target_z, target_w, target_g
            
            if update_visuals:
                self.entry_x.delete(0, tk.END); self.entry_x.insert(0, f"{self.curr_x:.1f}")
                self.entry_y.delete(0, tk.END); self.entry_y.insert(0, f"{self.curr_y:.1f}")
                self.entry_z.delete(0, tk.END); self.entry_z.insert(0, f"{self.curr_z:.1f}")
                if not self.is_playing and not self.is_moving:
                    self.status_var.set(f"Pos: ({self.curr_x:.1f}, {self.curr_y:.1f}, {self.curr_z:.1f}) | W: {self.curr_w}°")

            deg = np.rad2deg(joint_angles)
            s1 = int(max(0, min(180, deg[1] + 90)))
            s2 = int(max(0, min(180, 90 - deg[2])))
            s3 = int(max(0, min(180, deg[3] + 45)))
            s4_wrist = int(self.curr_w) 
            s4_claw = 110 if self.curr_g == GRIPPER_OPEN else 150

            # Multiply your floating-point angles by 10 and convert to integers
            msg = f"{int(s1 * 10)},{int(s2 * 10)},{int(s3 * 10)},{int(s4_wrist * 10)},{int(s4_claw * 10)}\n"
            if self.ser:
                self.ser.write(msg.encode())
        else:
            if update_visuals:
                self.status_var.set(f"UNREACHABLE! Error: {error_in:.2f} in")

        if update_visuals:
            self.update_plot(joint_angles, target_m, reachable)

    # def update_plot(self, angles, target, reachable):
    #         self.ax.clear()
    #         self.my_chain.plot(angles, self.ax, target=target)
    #         formatter = ticker.FuncFormatter(lambda x, pos: f'{x / INCH_TO_METERS:.1f}')
    #         self.ax.xaxis.set_major_formatter(formatter)
    #         self.ax.yaxis.set_major_formatter(formatter)
    #         self.ax.zaxis.set_major_formatter(formatter)
    #         self.ax.set_xlabel('X (in)'); self.ax.set_ylabel('Y (in)'); self.ax.set_zlabel('Z (in)')
    #         self.ax.set_title("REACHABLE" if reachable else "UNREACHABLE", color='green' if reachable else 'red')
    #         limit = 15 * INCH_TO_METERS
    #         self.ax.set_xlim(-limit, limit); self.ax.set_ylim(-limit, limit); self.ax.set_zlim(0, limit)
    #         self.ax.set_box_aspect([1,1,1])
    #         self.canvas.draw()
    
    def update_plot(self, angles, target, reachable):
        self.ax.clear()
        
        # --- 1. LET IKPY DRAW THE ARM ---
        self.my_chain.plot(angles, self.ax, target=target)

        # --- 2. HIJACK IKPY'S STYLING ---
        for line in self.ax.lines:
            line.set_linewidth(6)          # Chunky, solid links
            line.set_color('#0078D7')      # Windows/CAD vibrant blue
            line.set_solid_capstyle('round')

        for collection in self.ax.collections:
            collection.set_sizes([100])    # Much larger joint spheres
            collection.set_color('#333333')# Dark gray joints
            collection.set_edgecolor('white') # White outline for pop
            collection.set_linewidth(1.5)

        # --- 3. ADD A CAD "GLASS TABLE" & ORIGIN LINES ---
        limit = 15 * INCH_TO_METERS
        
        # Draw a semi-transparent baseplate on the floor (Z=0)
        xx, yy = np.meshgrid(np.linspace(-limit, limit, 2), np.linspace(-limit, limit, 2))
        zz = np.zeros_like(xx)
        self.ax.plot_surface(xx, yy, zz, color='#cccccc', alpha=0.15)

        # Draw dashed centerlines on the floor so the origin is obvious
        self.ax.plot([-limit, limit], [0, 0], [0, 0], color='#888888', linewidth=1.5, linestyle='--')
        self.ax.plot([0, 0], [-limit, limit], [0, 0], color='#888888', linewidth=1.5, linestyle='--')

        # --- 4. STYLE THE TARGET MARKER ---
        if target is not None:
            # Overwrite ikpy's default target with a high-vis neon crosshair/ring
            self.ax.scatter(target[0], target[1], target[2], 
                            color='#FF00FF', s=150, marker='+', linewidth=3, zorder=5)
            self.ax.scatter(target[0], target[1], target[2], 
                            color='none', edgecolor='#FF00FF', s=300, marker='o', linewidth=1.5, zorder=5)

        # --- 5. FLOATING TELEMETRY HUD ---
        # Converts back to inches for display inside the plot canvas
        if target is not None:
            hud_text = f" TARGET X: {target[0]/INCH_TO_METERS:>5.1f} in \n TARGET Y: {target[1]/INCH_TO_METERS:>5.1f} in \n TARGET Z: {target[2]/INCH_TO_METERS:>5.1f} in "
            self.ax.text2D(0.02, 0.98, hud_text, transform=self.ax.transAxes, 
                           color='#005500' if reachable else '#dd0000', 
                           fontname='Consolas', fontsize=10, weight='bold',
                           bbox=dict(facecolor='white', alpha=0.85, edgecolor='#cccccc', boxstyle='round,pad=0.4'),
                           verticalalignment='top')

        # --- 6. FORMAT THE AXES ---
        formatter = ticker.FuncFormatter(lambda x, pos: f'{x / INCH_TO_METERS:.1f}')
        self.ax.xaxis.set_major_formatter(formatter)
        self.ax.yaxis.set_major_formatter(formatter)
        self.ax.zaxis.set_major_formatter(formatter)

        # --- 7. AESTHETICS & VIEWING ANGLE ---
        self.ax.view_init(elev=25, azim=45) 
        
        # Make the background panes entirely invisible
        self.ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        self.ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        self.ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        
        # Dim the grid lines so they look like a subtle blueprint
        grid_color = (0.7, 0.7, 0.7, 0.4)
        self.ax.xaxis._axinfo["grid"]['color'] = grid_color
        self.ax.yaxis._axinfo["grid"]['color'] = grid_color
        self.ax.zaxis._axinfo["grid"]['color'] = grid_color

        # Clean typography for labels
        self.ax.set_xlabel('\nX Axis', fontname='Consolas', fontsize=9)
        self.ax.set_ylabel('\nY Axis', fontname='Consolas', fontsize=9)
        self.ax.set_zlabel('\nZ Axis', fontname='Consolas', fontsize=9)
        
        self.ax.set_title("REACHABLE" if reachable else "UNREACHABLE", 
                          color='#008800' if reachable else '#dd0000', 
                          weight='bold', fontname='Segoe UI', pad=10)

        # --- 8. LOCK THE CAMERA LIMITS & OPTIMIZE MARGINS ---
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_zlim(0, limit)
        self.ax.set_box_aspect([1, 1, 1])
        
        # Remove wasted whitespace around the 3D plot
        self.fig.tight_layout()

        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmApp(root)
    root.mainloop()