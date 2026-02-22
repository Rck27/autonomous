import socket
import time

def simulate_yolo_stream():
    UDP_IP = "127.0.0.1"
    UDP_PORT = 9000
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("Waiting 15 seconds to let the drone take off and start mission...")
    time.sleep(5)

    print("Object detected! Starting stream...")

    # Start coordinates (Object is at the top right of the screen)
    # dx: +0.8 (right), dy: -0.7 (top)
    dx = 0.9
    dy = -0.7

    # Simulate 10 frames per second
    for _ in range(200): # 6 seconds of data
        # Move coordinates closer to (0, 0) simulating the drone centering it
        if dx > 0.05: dx -= 0.02
        if dy < -0.05: dy += 0.02
        
        # If very close, snap to 0
        if abs(dx) < 0.06: dx = 0.0
        if abs(dy) < 0.06: dy = 0.0

        message = f"TARGET: {dx:.2f}, {dy:.2f}".encode()
        sock.sendto(message, (UDP_IP, UDP_PORT))
        print(f"Sent: {message.decode()}")
        
        time.sleep(0.1) # 10 Hz

    print("Simulation stream finished.")

if __name__ == "__main__":
    simulate_yolo_stream()