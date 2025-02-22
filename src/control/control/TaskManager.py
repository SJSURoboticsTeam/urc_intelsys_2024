import rclpy
import rclpy.node
import rclpy.executors
import constants
import control
import control.Task as Task
import json
import control.aruconav
import control.gpsnav
import control.objectnav
import rclpy.subscription


class TaskManager(rclpy.node.Node):
    def __init__(self):
        super().__init__("task_manager")
        self.create_timer(constants.UPDATE_FREQ, self.call)
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "task_types",
                    [1, 1],
                ),
                ("task_details", ["{}", "{}"]),
            ],
        )

        task_types = self.get_parameter("task_types").value
        task_details = self.get_parameter("task_details").value
        self.all_tasks = [
            Task.Task(Task.TaskType(tp), details)
            for tp, details in zip(task_types, task_details)
        ]
        self.get_logger().info(f"Tasks: {self.all_tasks}")
        self.current_task = 0
        self.gpsnav = control.gpsnav.GPSNav()
        self.objectnav = control.objectnav.ObjectNav()
        self.aruconav = control.aruconav.ArucoNav()
        self.task_map = {
            Task.TaskType.GPSNAV: self.gpsnav,
            Task.TaskType.OBJECT_NAV: self.objectnav,
            Task.TaskType.ARUCO_NAV: self.aruconav,
        }

    def call(self):
        task = self.all_tasks[self.current_task]
        tasktype, details = task.task, task.details
        ret = self.task_map[tasktype].call(**json.loads(details))
        if ret:  # advance to next task
            self.current_task += 1

    def nodes(self):
        return [self, self.gpsnav, self.objectnav, self.aruconav]


def main(args=None):
    rclpy.init(args=args)
    task_manager = TaskManager()
    try:
        while True:
            for node in task_manager.nodes():
                rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("shutting down")
