import rclpy.node


class GPSNav(rclpy.node.Node):
    def __init__(self):
        super().__init__("gpsnav")

    def call(self, *args, **kwargs):
        pass
