#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import os

# ANSI color codes for pretty output
class Colors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class InteractiveController(Node):
    def __init__(self):
        super().__init__('interactive_llm_controller')
        self.publisher = self.create_publisher(String, 'llm_command', 10)

        # Give time for publisher to initialize
        self.get_logger().info("Interactive LLM Controller ready!")

    def send_command(self, command_text):
        msg = String()
        msg.data = command_text
        self.publisher.publish(msg)
        print(f"{Colors.OKGREEN}✓ Sent to robot{Colors.ENDC}")

def print_banner():
    banner = f"""
{Colors.HEADER}{Colors.BOLD}╔═══════════════════════════════════════════════════════════╗
║                                                           ║
║        🤖 P3DX LLM Interactive Control Console 🤖         ║
║                                                           ║
║         Talk to your robot in natural language!           ║
║                                                           ║
╚═══════════════════════════════════════════════════════════╝{Colors.ENDC}

{Colors.OKCYAN}Just type what you want the robot to do, and press Enter.
The LLM will understand and execute your command!{Colors.ENDC}

{Colors.WARNING}Example commands:{Colors.ENDC}
  • move forward 2 meters
  • turn left 90 degrees
  • go backward slowly
  • rotate right 45 degrees
  • stop

{Colors.FAIL}Special commands:{Colors.ENDC}
  • {Colors.BOLD}emergency{Colors.ENDC} - Emergency stop (not implemented in this CLI, use service)
  • {Colors.BOLD}quit{Colors.ENDC} or {Colors.BOLD}exit{Colors.ENDC} - Exit this program

{Colors.OKBLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━{Colors.ENDC}
"""
    print(banner)

def main():
    rclpy.init()
    controller = InteractiveController()

    print_banner()

    try:
        while True:
            try:
                # Prompt for user input
                print(f"\n{Colors.BOLD}{Colors.OKCYAN}You:{Colors.ENDC} ", end='', flush=True)
                user_input = input().strip()

                # Handle empty input
                if not user_input:
                    continue

                # Check for exit commands
                if user_input.lower() in ['quit', 'exit', 'q']:
                    print(f"\n{Colors.WARNING}Goodbye! 👋{Colors.ENDC}\n")
                    break

                # Check for emergency stop
                if user_input.lower() in ['emergency', 'emergency stop', 'estop']:
                    print(f"{Colors.FAIL}⚠ Emergency stop not implemented in CLI.{Colors.ENDC}")
                    print(f"{Colors.FAIL}Use: ros2 service call /emergency_stop std_srvs/Empty{Colors.ENDC}")
                    continue

                # Send command
                print(f"{Colors.OKBLUE}🤖 Robot:{Colors.ENDC} Processing your command...", flush=True)
                controller.send_command(user_input)

                # Spin once to publish
                rclpy.spin_once(controller, timeout_sec=0.1)

            except KeyboardInterrupt:
                print(f"\n\n{Colors.WARNING}Interrupted! Press Ctrl+C again to exit, or type 'quit'{Colors.ENDC}")
                continue

    except KeyboardInterrupt:
        print(f"\n{Colors.WARNING}Exiting...{Colors.ENDC}\n")

    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
