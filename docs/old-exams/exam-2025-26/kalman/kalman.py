from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class KalmanFilter:
    # TODO: implement the __init__, predict, and update methods of the KalmanFilter class
    # Add parameters to the methods as needed, but DO NOT remove existing ones.
    def __init__(self, x: np.ndarray):
        self.x = x  # state vector, the first two elements MUST BE position (px, py)


    def predict(self) -> None:
        ...


    def update(self, z: np.ndarray) -> None:
        ...
