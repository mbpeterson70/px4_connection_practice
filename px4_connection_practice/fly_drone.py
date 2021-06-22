# ROS imports
import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import Timesync
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleRatesSetpoint

class FlyDroneNode(Node):

    def __init__(self):
        super().__init__('fly_drone')
        self.ARM_VEHICLE = 1.0
        self.now = 0
        self.arm_cnt = 0

        self.trajectory_pub = self.create_publisher(TrajectorySetpoint,
            'TrajectorySetpoint_PubSubTopic', 1)
        self.rates_pub = self.create_publisher(VehicleRatesSetpoint,
            'VehicleRatesSetpoint_PubSubTopic', 1)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand,
            'VehicleCommand_PubSubTopic', 1) #5)
        self.offboard_control_pub = self.create_publisher(OffboardControlMode,
            'OffboardControlMode_PubSubTopic', 1)
        self.time_sub = self.create_subscription(Timesync, 
            'Timesync_PubSubTopic', self.timeCallback, 10)

    def start_controller(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, self.ARM_VEHICLE)
        self.publish_offboard_control()

    def fly(self):
        trajectory_msg = TrajectorySetpoint()
        rates_msg = VehicleRatesSetpoint()

        trajectory_msg.timestamp = self.now
        # trajectory_msg.thrust = [0., 0., -.5]
        trajectory_msg.x = 2.0
        trajectory_msg.y = 2.0
        trajectory_msg.z = -3.0
        trajectory_msg.yaw = -3.14

        rates_msg.timestamp = self.now
        rates_msg.roll = 0.0
        rates_msg.pitch = 0.0
        rates_msg.yaw = 2.0

        self.trajectory_pub.publish(trajectory_msg)
        self.rates_pub.publish(rates_msg)

    def timeCallback(self, msg):
        if self.arm_cnt == 10:
            self.start_controller()
        if self.arm_cnt < 11:
            self.arm_cnt += 1
        self.now = msg.timestamp
        self.publish_offboard_control()
        self.fly()

    def publish_vehicle_command(self, command, param1 = 0.0, param2 = 0.0):
        vehicle_command_msg = VehicleCommand()
        vehicle_command_msg.timestamp = self.now
        vehicle_command_msg.param1 = float(param1)
        vehicle_command_msg.param2 = float(param2)
        vehicle_command_msg.command = command
        vehicle_command_msg.target_system = 1
        vehicle_command_msg.target_component = 1
        vehicle_command_msg.source_system = 1
        vehicle_command_msg.source_component = 1
        vehicle_command_msg.from_external = True
        self.vehicle_command_pub.publish(vehicle_command_msg)

    def publish_offboard_control(self):
        offboard_ctrl_msg = OffboardControlMode()
        offboard_ctrl_msg.timestamp = self.now
        offboard_ctrl_msg.position = True
        offboard_ctrl_msg.velocity = False
        offboard_ctrl_msg.acceleration = False
        offboard_ctrl_msg.attitude = False
        offboard_ctrl_msg.body_rate = True
        self.offboard_control_pub.publish(offboard_ctrl_msg)


def main(args=None):
    rclpy.init(args=args)
    fly_drone_node = FlyDroneNode()
    rclpy.spin(fly_drone_node)
    fly_drone_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
