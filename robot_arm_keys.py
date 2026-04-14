import ikpy.chain
import numpy as np
import serial
import time
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker 
from pynput import keyboard

# --- CONFIGURATION ---
URDF_FILE = "simple_arm.urdf"
PORT = 'COM6' 
BAUD = 115200
INCH_TO_METERS = 0.0254
TOLERANCE_INCHES = 0.5 
GRIPPER_OPEN = 1
GRIPPER_CLOSED = 0

# Jogging Settings
STEP_SIZE = 0.4  # Inches to move per key tap
curr_x, curr_y, curr_z = 5.0, 0.0, 5.0 # Starting "Home" position
curr_g = GRIPPER_CLOSED

# Load the URDF
my_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE)

# Initialize Serial (Uncomment when your Arduino is plugged in)
# Initialize Serial
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print("Waiting for Arduino to boot...")
    
    # Wait for the "READY" signal
    while True:
        if ser.in_waiting > 0:
            msg = ser.readline().decode('utf-8').strip()
            if msg == "READY":
                print("Arduino is online and ready!")
                break
                
except Exception as e:
    print(f"Serial Error: {e}. Running in Simulation Mode.")
    ser = None

# --- SETUP INTERACTIVE PLOT ---
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def move_to_coords_inches(x_in, y_in, z_in, gripper_val=None):
    # 1. Convert to Meters
    target_m = [x_in * INCH_TO_METERS, y_in * INCH_TO_METERS, z_in * INCH_TO_METERS]

    # 2. Solve Inverse Kinematics (IK)
    joint_angles = my_chain.inverse_kinematics(target_m)

    # 3. Check Distance Error (Forward Kinematics)
    computed_pos_m = my_chain.forward_kinematics(joint_angles)[:3, 3]
    error_m = np.linalg.norm(computed_pos_m - target_m)
    error_in = error_m / INCH_TO_METERS
    
    reachable = error_in <= TOLERANCE_INCHES

    # 4. Map Radians to Servo Degrees (0-180)
    deg = np.rad2deg(joint_angles)
    s1_base = int(max(0, min(180, deg[1] + 90)))
    s2_shoulder = int(max(0, min(180, 90 - deg[2])))
    s3_elbow = int(max(0, min(180, deg[3] + 45)))
    s4_wrist = 90 # Default straight
    
    # Handle Claw State
    if gripper_val == GRIPPER_OPEN:
        s4_claw = 110  # Open position
    else:
        s4_claw = 150 # Closed position

    final_servos = [s1_base, s2_shoulder, s3_elbow, s4_wrist, s4_claw]
    command = ",".join(map(str, final_servos)) + "\n"

    # 5. Handle Movement & Simulation
    if not reachable:
        print(f"❌ UNREACHABLE: Target off by {error_in:.2f}\" | Pos: ({x_in:.1f}, {y_in:.1f}, {z_in:.1f})")
    else:
        print(f"✅ REACHED! Error: {error_in:.4f}\" | Pos: ({x_in:.1f}, {y_in:.1f}, {z_in:.1f}) | Sending: {command.strip()}")
        if ser: 
            ser.write(command.encode()) # Send to physical arm if serial is available

    plot_arm(joint_angles, target_m, reachable, error_in)
    return reachable

def plot_arm(angles, target, reachable, error_in):
    ax.clear() # Clear previous frame
    my_chain.plot(angles, ax, target=target)

    # Convert axes labels back to inches
    formatter = ticker.FuncFormatter(lambda x, pos: f'{x / INCH_TO_METERS:.1f}')
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    ax.zaxis.set_major_formatter(formatter)

    ax.set_xlabel('X (inches)')
    ax.set_ylabel('Y (inches)')
    ax.set_zlabel('Z (inches)')

    title_color = 'green' if reachable else 'red'
    status = "REACHABLE" if reachable else "UNREACHABLE"
    ax.set_title(f"Status: {status} | Error: {error_in:.2f}\"", color=title_color)

    limit_m = 15 * INCH_TO_METERS
    ax.set_xlim(-limit_m, limit_m)
    ax.set_ylim(-limit_m, limit_m)
    ax.set_zlim(0, limit_m)
    ax.set_box_aspect([1,1,1])
    
    plt.draw()
    plt.pause(0.01) # Briefly pause to update the UI without blocking

# --- STATE VARIABLES ---
curr_x, curr_y, curr_z = 5.0, 0.0, 5.0 # Actual safe position
cmd_x, cmd_y, cmd_z = 5.0, 0.0, 5.0    # Commanded position
curr_g = GRIPPER_CLOSED
cmd_g = GRIPPER_CLOSED
needs_update = True # Force initial draw

# --- JOG CONTROLLER (Background Thread) ---
def on_press(key):
    global cmd_x, cmd_y, cmd_z, cmd_g, needs_update
    
    try:
        if key.char == 'w': cmd_x += STEP_SIZE
        elif key.char == 's': cmd_x -= STEP_SIZE
        elif key.char == 'a': cmd_y -= STEP_SIZE
        elif key.char == 'd': cmd_y += STEP_SIZE
        elif key.char == 'o': cmd_g = GRIPPER_OPEN
        elif key.char == 'c': cmd_g = GRIPPER_CLOSED
            
    except AttributeError:
        if key == keyboard.Key.up: cmd_z += STEP_SIZE
        elif key == keyboard.Key.down: cmd_z -= STEP_SIZE
        elif key == keyboard.Key.esc: 
            # Stop listener and exit
            return False 

    needs_update = True

# --- MAIN EXECUTION (Main Thread) ---
if __name__ == "__main__":
    print("\n--- JOG MODE ACTIVE ---")
    print("W/S: X | A/D: Y | UP/DOWN: Z | O/C: Gripper | ESC: Quit")
    
    # Start the listener in the background (Notice we use .start(), not .join())
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    # The Main Loop handles all Matplotlib drawing
    while listener.running:
        if needs_update:
            # Try to move to the new commanded position
            reachable = move_to_coords_inches(cmd_x, cmd_y, cmd_z, cmd_g)
            
            if reachable:
                # Success! Update our current state
                curr_x, curr_y, curr_z, curr_g = cmd_x, cmd_y, cmd_z, cmd_g
            else:
                # Unreachable! Revert the commanded position back to safe actual
                cmd_x, cmd_y, cmd_z, cmd_g = curr_x, curr_y, curr_z, curr_g
                
            needs_update = False
        
        # plt.pause() keeps the GUI responsive and must be in the main thread!
        plt.pause(0.05)