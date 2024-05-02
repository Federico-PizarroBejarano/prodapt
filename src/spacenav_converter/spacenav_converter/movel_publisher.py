from rclpy.node import Node
from std_msgs.msg import String


class MovelPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher = self.create_publisher(
            String, "/urscript_interface/script_command", 10
        )

    def send_action(self, action):
        urscript_msg = String()
        urscript_msg.data = """
def my_prog():

    set_digital_out(1, True)

    movel(p[{0}, {1}, {2}, {3}, {4}, {5}], a=1.2, v=0.25, r=0)

end""".format(
            *action
        )

        self.publisher.publish(urscript_msg)
