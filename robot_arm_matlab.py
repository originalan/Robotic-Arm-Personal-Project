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

# Load the URDF
my_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE)

# Initialize Serial (Uncomment when your Arduino is plugged in)
ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

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
    s4_claw = 150 # Default closed

    # Override claw position if gripper_val is provided
    if gripper_val is not None:
        if (gripper_val == GRIPPER_OPEN):
            s4_claw = 110  # Open position
        else:
            s4_claw = 150 # Closed position

    final_servos = [s1_base, s2_shoulder, s3_elbow, s4_wrist, s4_claw]
    command = ",".join(map(str, final_servos)) + "\n"

    # 5. Handle Movement & Simulation
    if not reachable:
        print(f"❌ UNREACHABLE: Target is off by {error_in:.2f} inches.")
        print(f"   Showing 'best effort' in plot. Hardware movement blocked.")
    else:
        print(f"✅ REACHED! Error: {error_in:.4f}\". Sending: {command.strip()}")
        ser.write(command.encode()) # Send to physical arm

    print(f"   Joint Angles (degrees): Base={s1_base}, Shoulder={s2_shoulder}, Elbow={s3_elbow}, Wrist={s4_wrist}, Claw={s4_claw}")
    plot_arm(joint_angles, target_m, reachable, error_in)

def plot_arm(angles, target, reachable, error_in):
    # Create the figure
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
    my_chain.plot(angles, ax, target=target)

    # Convert axes labels back to inches for our human brains
    formatter = ticker.FuncFormatter(lambda x, pos: f'{x / INCH_TO_METERS:.1f}')
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    ax.zaxis.set_major_formatter(formatter)

    ax.set_xlabel('X (inches)')
    ax.set_ylabel('Y (inches)')
    ax.set_zlabel('Z (inches)')

    # Visual Feedback
    title_color = 'green' if reachable else 'red'
    status = "REACHABLE" if reachable else "UNREACHABLE"
    ax.set_title(f"Status: {status} | Error: {error_in:.2f}\"", color=title_color)

    # Set consistent scale
    limit_m = 15 * INCH_TO_METERS
    ax.set_xlim(-limit_m, limit_m)
    ax.set_ylim(-limit_m, limit_m)
    ax.set_zlim(0, limit_m)
    ax.set_box_aspect([1,1,1])
    
    print("Close the plot window to enter new coordinates...")
    plt.show()

# --- MAIN LOOP ---
if __name__ == "__main__":
    print("Standalone Robot Controller Online.")
    print("Type 'exit' to quit.")

    while True:
        user_input = input("\nEnter X, Y, Z (inches) and optional G (1=open, 0=close): ")
        
        if user_input.lower() == 'exit':
            break
            
        try:
            parts = [val.strip() for val in user_input.split(',')]
        
            # Extract X, Y, Z
            x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
            
            # Check for optional 4th value (the gripper)
            g_angle = None
            if len(parts) == 4:
                g_state = int(parts[3])
                g_angle = GRIPPER_OPEN if g_state == 1 else GRIPPER_CLOSED
            
            move_to_coords_inches(x, y, z, gripper_val=g_angle)
        except ValueError:
            print("Error: Invalid numbers. Try again.")