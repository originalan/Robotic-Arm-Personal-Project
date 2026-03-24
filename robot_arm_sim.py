import ikpy.chain
import numpy as np
import serial
import time
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker 
from mpl_toolkits.mplot3d import Axes3D
import socket

# Setup UDP Connection
UDP_IP = "127.0.0.1" # Standard local address
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
listen_sock.bind(("127.0.0.1", 5006))
listen_sock.setblocking(False) # Don't wait forever for a message

URDF_FILE = "simple_arm.urdf"
PORT = 'COM6' 
BAUD = 115200
INCH_TO_METERS = 0.0254
TOLERANCE_INCHES = 0.5 # Error allowed before we say "unreachable"

claw_open = False

# Load the URDF
my_chain = ikpy.chain.Chain.from_urdf_file(URDF_FILE)

# Initialize Serial
# ser = serial.Serial(PORT, BAUD, timeout=1)
# time.sleep(2)

def send_to_unity(angles, target_m, reachable):
    # Format the angles as a simple string: "90,45,120"
    # then target string as "x,y,z" in meters for Unity to read
    status = 1 if not reachable else 0
    data_list = list(angles) + list(target_m) + [status]

    full_message = ",".join(map(str, data_list))
    
    sock.sendto(full_message.encode(), (UDP_IP, UDP_PORT))

def move_to_coords_inches(x_in, y_in, z_in):
    # 1. Convert Input Inches to Meters for the IK Solver
    target_m = [x_in * INCH_TO_METERS, y_in * INCH_TO_METERS, z_in * INCH_TO_METERS]

    # 2. Solve Inverse Kinematics
    joint_angles = my_chain.inverse_kinematics(target_m)

    # 3. DISTANCE CHECK (Forward Kinematics)
    # Calculate where the arm tip actually is based on those angles
    computed_pos_m = my_chain.forward_kinematics(joint_angles)[:3, 3]
    error_m = np.linalg.norm(computed_pos_m - target_m)
    error_in = error_m / INCH_TO_METERS
    reachable = error_in <= TOLERANCE_INCHES

    # 4. MAP INDIVIDUAL JOINTS (The "Secret Sauce")
    # Convert radians to degrees first
    deg = np.rad2deg(joint_angles)

    # Joint 1: Base (-90 to 90) -> Offset by 90 to get 0-180
    s1_base = deg[1] + 90
    
    # Joint 2: Shoulder (-90 to 90) -> Offset by 90 to get 0-180
    s2_shoulder = deg[2] + 90
    
    # Joint 3: Elbow (-45 to 135) -> Offset by 45 to get 0-180
    # (Because -45 + 45 = 0 degrees at the servo)
    s3_elbow = deg[3] + 45

    # Joint 4: Claw (Optional, since Wrist is 'fixed' in your URDF)
    s4_claw = 150 if claw_open else 30 

    # 5. CLAMP AND PACKAGE
    # We apply the clamp to each specifically to prevent buzzing/damage
    final_servos = [
        int(max(0, min(180, s1_base))),
        int(max(0, min(180, s2_shoulder))),
        int(max(0, min(180, s3_elbow)))
        # int(s4_claw)
    ]

    real_time_servos = [
        final_servos[0],
        final_servos[1],
        final_servos[2],
        30 if claw_open else 150
    ]

    # 6. Send to Arduino: "base,shoulder,elbow,claw\n"
    command = ",".join(map(str, real_time_servos)) + "\n"
    if not reachable:
        print(f"SIMULATION ONLY: Target off by {error_in:.2f}\". Hardware movement blocked.")
    else:
        print(f"REACHED Error: {error_in:.4f}\". Sending: {command.strip()}")
        # ser.write(command.encode()) 
    send_to_unity(final_servos, target_m, reachable)

    # plot_arm(joint_angles, target_m, reachable, error_in)
    return True

def plot_arm(angles, target, reachable, error_in):
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
    my_chain.plot(angles, ax, target=target)

    formatter = ticker.FuncFormatter(lambda x, pos: f'{x / INCH_TO_METERS:.1f}')
    ax.xaxis.set_major_formatter(formatter)
    ax.yaxis.set_major_formatter(formatter)
    ax.zaxis.set_major_formatter(formatter)

    ax.set_xlabel('X (inches)')
    ax.set_ylabel('Y (inches)')
    ax.set_zlabel('Z (inches)')

    title_color = 'green' if reachable else 'red'
    status = "REACHABLE" if reachable else "UNREACHABLE"
    ax.set_title(f"Status: {status} | Error: {error_in:.2f} inches", color=title_color)

    # Set plot limits to roughly 15 inches (in meters) for scale
    limit_m = 15 * INCH_TO_METERS
    ax.set_xlim(-limit_m, limit_m)
    ax.set_ylim(-limit_m, limit_m)
    ax.set_zlim(0, limit_m)
    ax.set_box_aspect([1,1,1])
    plt.show()

# --- TEST RUN ---
# in inches
# move_to_coords_inches(3, -3, 6)

def main():
    print("Real-time Unity Link Active...")
    try:
        while True:
            try:
                # Check if there is a message from Unity
                data, addr = listen_sock.recvfrom(1024)
                message = data.decode()
                
                # Parse "x,y,z" from Unity (already in meters)
                tx, ty, tz = [float(val) for val in message.split(',')]
                
                # Convert meters to inches for your move function 
                # (or just call ikpy directly with meters)
                target_m = [tx, ty, tz]
                
                # 1. Solve IK
                joint_angles = my_chain.inverse_kinematics(target_m)

                actual_pos_m = my_chain.forward_kinematics(joint_angles)[:3, 3]
                distance_error_m = np.linalg.norm(actual_pos_m - target_m)

                reachable = distance_error_m <= (TOLERANCE_INCHES * INCH_TO_METERS)
                
                # 2. Process Servos (reuse your existing logic)
                deg = np.rad2deg(joint_angles)
                s1 = int(max(0, min(180, deg[1] + 90)))
                s2 = int(max(0, min(180, deg[2] + 90)))
                s3 = int(max(0, min(180, deg[3] + 45)))
                
                # 3. Send back to Unity for the arm to move
                send_to_unity([s1, s2, s3], target_m, reachable)
                
                # 4. Optional: Send to Arduino
                # command = f"{s1},{s2},{s3}\n"
                # ser.write(command.encode())

            except BlockingIOError:
                # No message yet, just keep looping
                pass
            except Exception as e:
                print(f"Loop Error: {e}")
                
            time.sleep(0.01) # Small sleep to save CPU
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__ == "__main__":
    main()