import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

# Add the path to the folder containing xdpchandler.py
sys.path.append(os.path.abspath('/home/jay/ros2_movella/src/movella_pkg/movella_pkg/examples/xdpcsdk/python'))
from xdpchandler import XdpcHandler
import movelladot_pc_sdk  # Assuming you have this imported

class MovellaNode(Node):
    def __init__(self):
        super().__init__('movella_node')  # Initialize the ROS 2 node with a name

        self.logger = get_logger(self.get_name())  # Get the logger for the node
        self.xdpcHandler = XdpcHandler()

        if not self.xdpcHandler.initialize():
            self.xdpcHandler.cleanup()
            self.get_logger().error('Failed to initialize the XdpcHandler. Exiting...')
            rclpy.shutdown()
            return

        self.xdpcHandler.scanForDots()
        if len(self.xdpcHandler.detectedDots()) == 0:
            self.get_logger().error("No Movella DOT device(s) found. Aborting.")
            self.xdpcHandler.cleanup()
            rclpy.shutdown()
            return

        self.xdpcHandler.connectDots()
        if len(self.xdpcHandler.connectedDots()) == 0:
            self.get_logger().error("Could not connect to any Movella DOT device(s). Aborting.")
            self.xdpcHandler.cleanup()
            rclpy.shutdown()
            return

        # Setting filter profile and logging configuration
        for device in self.xdpcHandler.connectedDots():
            filterProfiles = device.getAvailableFilterProfiles()
            self.get_logger().info("Available filter profiles:")
            for f in filterProfiles:
                self.get_logger().info(f.label())

            self.get_logger().info(f"Current profile: {device.onboardFilterProfile().label()}")
            if device.setOnboardFilterProfile("General"):
                self.get_logger().info("Successfully set profile to General")
            else:
                self.get_logger().error("Setting filter profile failed!")

            self.get_logger().info("Setting quaternion CSV output")
            device.setLogOptions(movelladot_pc_sdk.XsLogOptions_Quaternion)

            logFileName = "logfile_" + device.bluetoothAddress().replace(':', '-') + ".csv"
            self.get_logger().info(f"Enable logging to: {logFileName}")
            if not device.enableLogging(logFileName):
                self.get_logger().error(f"Failed to enable logging. Reason: {device.lastResultText()}")

            self.get_logger().info("Putting device into measurement mode.")
            if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_ExtendedEuler):
                self.get_logger().error(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
                continue

        self.get_logger().info("\nMain loop. Recording data for 10 seconds.")
        self.record_data()

    def record_data(self):
        # Record data for 10 seconds
        orientationResetDone = False
        startTime = movelladot_pc_sdk.XsTimeStamp_nowMs()
        
        while movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime <= 10000:
            if self.xdpcHandler.packetsAvailable():
                s = ""
                for device in self.xdpcHandler.connectedDots():
                    packet = self.xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())

                    if packet.containsOrientation():
                        euler = packet.orientationEuler()
                        s += f"Roll:{euler.x():7.2f}, Pitch:{euler.y():7.2f}, Yaw:{euler.z():7.2f}| "

                self.get_logger().info(s)

                if not orientationResetDone and movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime > 5000:
                    for device in self.xdpcHandler.connectedDots():
                        self.get_logger().info(f"\nResetting heading for device {device.portInfo().bluetoothAddress()}: ", end="")
                        if device.resetOrientation(movelladot_pc_sdk.XRM_Heading):
                            self.get_logger().info("OK")
                        else:
                            self.get_logger().error(f"NOK: {device.lastResultText()}")
                    orientationResetDone = True

        self.stop_measurement()

    def stop_measurement(self):
        self.get_logger().info("\nStopping measurement...")
        for device in self.xdpcHandler.connectedDots():
            if not device.stopMeasurement():
                self.get_logger().error("Failed to stop measurement.")
            if not device.disableLogging():
                self.get_logger().error("Failed to disable logging.")

        self.xdpcHandler.cleanup()

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library

    node = MovellaNode()

    rclpy.spin(node)  # Keep the node running until it is stopped or interrupted

    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':
    main()
