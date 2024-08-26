from rclpy.node import Node
from std_msgs.msg import String

from prodapt.utils.rotation_utils import rotation_6d_to_axis_angle


class MovelPublisher(Node):
    def __init__(self, action_list, base_command):
        super().__init__("movel_publisher")
        self.action_list = action_list
        self.base_command = base_command

        self.publisher = self.create_publisher(
            String, "/urscript_interface/script_command", 10
        )

    def send_action(self, action, **kwargs):
        applied_action = self.base_command.copy()
        if "commanded_ee_position" in self.action_list:
            applied_action[:3] = action[:3]
        if "commanded_ee_position_xy" in self.action_list:
            applied_action[:2] = action[:2]
        if "commanded_ee_rotation_6d" in self.action_list:
            axis_angle = rotation_6d_to_axis_angle(action[3:])
            applied_action[3:] = axis_angle

        urscript_msg = String()
        urscript_msg.data = """
def my_prog():

    movel(p[{0}, {1}, {2}, {3}, {4}, {5}], a=1.2, v=0.25, r=0)

end""".format(
            *applied_action
        )

        self.publisher.publish(urscript_msg)
