from dataclasses import dataclass
from enum import Enum


class TaskType(Enum):
    GPSNAV = 1
    OBJECT_NAV = 2
    ARUCO_NAV = 3


@dataclass
class Task:
    task: TaskType
    details: str  # should be json loadable
