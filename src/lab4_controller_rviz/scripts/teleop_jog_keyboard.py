#!/usr/bin/env python3
"""
LAB4 Standalone Teleop Keyboard
This will definitely work independently of package issues
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, termios, tty
import threading
import time

msg = """
======================================
LAB4 TELEOPERATION KEYBOARD
======================================
Control Your Robot!

Linear Motion:          Angular Motion:
   W                       U   I   O
 A S D                     J   K   L
 Q   E

W/S: Forward/Back (X)   U/O: Yaw (Z)
A/D: Left/Right (Y)     I/K: Roll (X)  
Q/E: Up/Down (Z)        J/L: Pitch (Y)

F: Toggle Frame (World â†” End-Effector)
SPACE: Emergency Stop
ESC: Quit

HOLD keys for continuous motion!
======================================
"""

class TeleopKeyboardLAB4(Node):
    def __init__(self):
        super().__init__('teleop_keyboard_lab4')
        
        # Parameters - INCREASED for better responsiveness
        self.linear_speed = 0.1  # m/s (was 0.05)
        self.angular_speed = 0.5  # rad/s (was 0.3)
        
        # State
        self.velocity_frame = 'world'
        self.twist = Twist()
        self.running = True
        
        # Publishers
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.frame_pub = self.create_publisher(String, '/velocity_frame', 10)
        
        # Publish initial frame
        self._publish_frame()
        
        # Timer for continuous publishing
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz
        
        print(msg)
        print(f"Current Frame: {self.velocity_frame.upper()}")
        print(f"Linear Speed: {self.linear_speed} m/s")
        print(f"Angular Speed: {self.angular_speed} rad/s")
        print("-" * 40)
        
        self.get_logger().info('Teleop keyboard ready!')
        
    def timer_callback(self):
        """Continuously publish twist message"""
        self.twist_pub.publish(self.twist)
    
    def _publish_frame(self):
        """Publish current frame setting"""
        msg = String()
        msg.data = self.velocity_frame
        self.frame_pub.publish(msg)
        # Publish multiple times to ensure reception
        for _ in range(3):
            self.frame_pub.publish(msg)
            time.sleep(0.01)
    
    def get_key(self, timeout=0.1):
        """Get keyboard input with timeout"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        """Main keyboard control loop"""
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            print("Ready for input. Press keys to move...")
            last_key_time = time.time()
            
            while self.running and rclpy.ok():
                key = self.get_key(0.1)
                
                if key:
                    last_key_time = time.time()
                    
                    # Create new twist
                    new_twist = Twist()
                    
                    # Linear controls
                    if key == 'w':
                        new_twist.linear.x = self.linear_speed
                        print(f"\r[W] Forward: vx={self.linear_speed:.3f} m/s     ", end='')
                    elif key == 's':
                        new_twist.linear.x = -self.linear_speed
                        print(f"\r[S] Backward: vx={-self.linear_speed:.3f} m/s    ", end='')
                    elif key == 'a':
                        new_twist.linear.y = self.linear_speed
                        print(f"\r[A] Left: vy={self.linear_speed:.3f} m/s         ", end='')
                    elif key == 'd':
                        new_twist.linear.y = -self.linear_speed
                        print(f"\r[D] Right: vy={-self.linear_speed:.3f} m/s       ", end='')
                    elif key == 'q':
                        new_twist.linear.z = self.linear_speed
                        print(f"\r[Q] Up: vz={self.linear_speed:.3f} m/s           ", end='')
                    elif key == 'e':
                        new_twist.linear.z = -self.linear_speed
                        print(f"\r[E] Down: vz={-self.linear_speed:.3f} m/s        ", end='')
                    
                    # Angular controls
                    elif key == 'u':
                        new_twist.angular.z = self.angular_speed
                        print(f"\r[U] Yaw+: wz={self.angular_speed:.3f} rad/s      ", end='')
                    elif key == 'o':
                        new_twist.angular.z = -self.angular_speed
                        print(f"\r[O] Yaw-: wz={-self.angular_speed:.3f} rad/s     ", end='')
                    elif key == 'i':
                        new_twist.angular.x = self.angular_speed
                        print(f"\r[I] Roll+: wx={self.angular_speed:.3f} rad/s     ", end='')
                    elif key == 'k':
                        new_twist.angular.x = -self.angular_speed
                        print(f"\r[K] Roll-: wx={-self.angular_speed:.3f} rad/s    ", end='')
                    elif key == 'j':
                        new_twist.angular.y = self.angular_speed
                        print(f"\r[J] Pitch+: wy={self.angular_speed:.3f} rad/s    ", end='')
                    elif key == 'l':
                        new_twist.angular.y = -self.angular_speed
                        print(f"\r[L] Pitch-: wy={-self.angular_speed:.3f} rad/s   ", end='')
                    
                    # Control keys
                    elif key == 'f':
                        self.velocity_frame = 'end_effector' if self.velocity_frame == 'world' else 'world'
                        self._publish_frame()
                        symbol = 'ðŸŒ' if self.velocity_frame == 'world' else 'ðŸ”§'
                        print(f"\n{symbol} Frame switched to: {self.velocity_frame.upper()}")
                        self.get_logger().info(f'Frame: {self.velocity_frame}')
                    elif key == ' ':
                        new_twist = Twist()  # All zeros
                        print("\r[SPACE] EMERGENCY STOP!                           ")
                    elif key == '\x1b':  # ESC
                        print("\n\nExiting...")
                        self.running = False
                        break
                    else:
                        # Unknown key - don't change velocity
                        continue
                    
                    # Update twist
                    self.twist = new_twist
                    sys.stdout.flush()
                
                else:
                    # No key pressed - auto-stop after timeout
                    if time.time() - last_key_time > 0.3:
                        if any([
                            abs(self.twist.linear.x) > 0,
                            abs(self.twist.linear.y) > 0,
                            abs(self.twist.linear.z) > 0,
                            abs(self.twist.angular.x) > 0,
                            abs(self.twist.angular.y) > 0,
                            abs(self.twist.angular.z) > 0
                        ]):
                            self.twist = Twist()
                            print("\r[STOPPED] No input                               ", end='')
                            sys.stdout.flush()
        
        except Exception as e:
            print(f"\nError: {e}")
        
        finally:
            # Cleanup
            self.twist = Twist()
            self.twist_pub.publish(self.twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    
    teleop = TeleopKeyboardLAB4()
    
    # Create spinner thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop,), daemon=True)
    spin_thread.start()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    
    teleop.destroy_node()
    rclpy.shutdown()
    print("\nTeleop stopped.")

if __name__ == '__main__':
    main()