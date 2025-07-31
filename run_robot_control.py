import time
import signal
# our imports
from franka_robot import FrankaRobot
from game_controller import GameController
import numpy as np


class RobotController:
    def __init__(self):
        self.running = True
        self.joy = GameController()
        self.robot = FrankaRobot()
        self.robot.robot.recover_from_errors()

        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C and other termination signals"""
        print("\nShutdown signal received. Stopping robot gracefully...")
        self.running = False

    def run(self):
        """Main control loop with proper error handling and timing"""
        try:
            self.robot.move_home()
            print("Beginning control loop (Press Ctrl+C to exit gracefully)")

            # Control loop timing
            target_frequency = 100  # Hz
            target_dt = 1.0 / target_frequency

            while self.running:
                loop_start = time.time()

                # print(self.robot.robot.current_joint_positions)

                try:
                    # Get controller inputs
                    robot_inputs = self.joy.get_robot_inputs_from_controller()

                    # Send commands to robot
                    self.robot.move_velocity_inputs(robot_inputs)

                except Exception as e:
                    print(f"Exception occurred while moving robot: {e}")
                    try:
                        self.robot.robot.recover_from_errors()
                    except Exception as recovery_error:
                        print(
                            f"Failed to recover from errors: {recovery_error}")
                        self.running = False
                        break

                # Maintain loop timing
                elapsed = time.time() - loop_start
                sleep_time = target_dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    print(
                        f"Warning: Loop took {elapsed:.4f}s, target was {target_dt:.4f}s")

        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        except Exception as e:
            print(f"Fatal error in main loop: {e}")
        finally:
            self.shutdown()

    def shutdown(self):
        """Clean shutdown procedure"""
        print("Shutting down robot controller...")
        try:
            # Stop the robot safely
            self.robot.robot.stop()
            print("Robot stopped successfully")
        except Exception as e:
            print(f"Error stopping robot: {e}")
        print("Shutdown complete")


if __name__ == "__main__":
    controller = RobotController()
    controller.run()
