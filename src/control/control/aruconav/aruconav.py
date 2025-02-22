import rclpy.node


class ArucoNav(rclpy.node.Node):
    def __init__(self):
        super().__init__("aruconav")

    def call(self, *args, **kwargs):
        raise RuntimeError("ArucoNav.call not implemented yet!")
