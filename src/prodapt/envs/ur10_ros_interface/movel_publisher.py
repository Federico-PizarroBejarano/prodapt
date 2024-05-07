import time

from rclpy.node import Node
from std_msgs.msg import String

from prodapt.utils.rotation_utils import rotation_6d_to_axis_angle


class MovelPublisher(Node):
    def __init__(self):
        super().__init__("movel_publisher")
        self.publisher = self.create_publisher(
            String, "/urscript_interface/script_command", 10
        )

    def send_action(self, action, duration, **kwargs):
        urscript_msg = String()
        axis_angle = rotation_6d_to_axis_angle(action[3:])
        urscript_msg.data = """
def my_prog():

    movel(p[{0}, {1}, {2}, {3}, {4}, {5}], a=1.2, v=0.25, r=0)

end""".format(
            *action[:3], *axis_angle
        )

        self.publisher.publish(urscript_msg)
        time.sleep(duration)
