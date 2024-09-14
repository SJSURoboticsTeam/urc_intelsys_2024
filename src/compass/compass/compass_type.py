from abc import ABC, abstractmethod


class Compass_(ABC):
    @abstractmethod
    def get_cur_angle(self) -> float:
        """
        Returns the current angle from North in degrees.

        @return - float - a float from [0, 360) representing the current clockwise
            angle from North.
        """
