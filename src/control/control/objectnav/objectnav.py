import rclpy.node


class ObjectNav(rclpy.node.Node):
    def __init__(self):
        super().__init__("objectnav")

    def call(self, *args, **kwargs):
        raise RuntimeError("ObjectNav.call not implemented yet!")
