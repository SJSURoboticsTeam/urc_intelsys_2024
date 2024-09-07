from typing import List
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


class TaskManager(rclpy.node.Node):
    def __init__(self, tasks: List[Task.Task] = []):
        super().__init__("task_manager")
        self.create_timer(constants.UPDATE_FREQ, self.call)

        self.all_tasks = tasks
        self.current_task = 0
        self.task_map = {
            Task.TaskType.GPSNAV: control.gpsnav.GPSNav(),
            Task.TaskType.OBJECT_NAV: control.objectnav.ObjectNav(),
            Task.TaskType.ARUCO_NAV: control.aruconav.ArucoNav(),
        }

    def call(self):
        task = self.all_tasks[self.current_task]
        tasktype, details = task.task, task.details
        ret = self.task_map[tasktype].call(json.loads(details))
        if ret:  # advance to next task
            self.current_task += 1
