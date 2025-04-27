import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import String  # Publish JSON as String messages
import json

# Add the path to xdpchandler.py
sys.path.append(os.path.abspath('/home/jay/ros2_movella/src/movella_pkg/movella_pkg/examples/xdpcsdk/python'))


from xdpchandler import XdpcHandler
import movelladot_pc_sdk  # Movella SDK


class MovellaNode(Node):
    def __init__(self):
        super().__init__('movella_node')

        self.logger = get_logger(self.get_name())
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

        # Create publishers dynamically
        self.IMU_publishers = {}

        for idx, device in enumerate(self.xdpcHandler.connectedDots(), start=1):
            topic_name = f"imu_sensor_{idx}"
            publisher = self.create_publisher(String, topic_name, 10)

            # Store publisher per device
            self.IMU_publishers[device.portInfo().bluetoothAddress()] = publisher
            self.get_logger().info(f"Created publisher on topic: {topic_name}")

            # Set device settings
            filterProfiles = device.getAvailableFilterProfiles()
            for f in filterProfiles:
                self.get_logger().info(f"Available filter profile: {f.label()}")
            if device.setOnboardFilterProfile("General"):
                self.get_logger().info("Successfully set profile to General")
            else:
                self.get_logger().error("Setting filter profile failed!")

            self.get_logger().info("Putting device into measurement mode.")
            if not device.startMeasurement(movelladot_pc_sdk.XsPayloadMode_ExtendedEuler):
                self.get_logger().error(f"Could not put device into measurement mode. Reason: {device.lastResultText()}")
                continue

        self.get_logger().info("Main loop. Continuously recording and publishing IMU data.")
        self.record_data()

    def record_data(self):
        orientationResetDone = False
        startTime = movelladot_pc_sdk.XsTimeStamp_nowMs()

        while rclpy.ok():
            if self.xdpcHandler.packetsAvailable():
                for device in self.xdpcHandler.connectedDots():
                    packet = self.xdpcHandler.getNextPacket(device.portInfo().bluetoothAddress())

                    if packet.containsOrientation():
                        euler = packet.orientationEuler()
                        imu_data = {
                            "Roll": round(euler.x(), 2),
                            "Pitch": round(euler.y(), 2),
                            "Yaw": round(euler.z(), 2)
                        }

                        # Publish JSON string
                        publisher = self.IMU_publishers.get(device.portInfo().bluetoothAddress())
                        if publisher:
                            msg = String()
                            msg.data = json.dumps(imu_data)
                            publisher.publish(msg)

                # Reset orientation once after 5 seconds
                if not orientationResetDone and movelladot_pc_sdk.XsTimeStamp_nowMs() - startTime > 5000:
                    for device in self.xdpcHandler.connectedDots():
                        addr = device.portInfo().bluetoothAddress()
                        self.get_logger().info(f"Resetting heading for device {addr}:")
                        if device.resetOrientation(movelladot_pc_sdk.XRM_Heading):
                            self.get_logger().info("OK")
                        else:
                            self.get_logger().error(f"NOK: {device.lastResultText()}")
                    orientationResetDone = True

    def stop_measurement(self):
        self.get_logger().info("Stopping measurement...")
        for device in self.xdpcHandler.connectedDots():
            if not device.stopMeasurement():
                self.get_logger().error("Failed to stop measurement.")
        self.xdpcHandler.cleanup()

def main(args=None):
    rclpy.init(args=args)

    node = MovellaNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_measurement()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
