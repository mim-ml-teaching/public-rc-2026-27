from abc import ABC, abstractmethod
from typing import Tuple, Optional

import cv2
import numpy as np

from utils import floats_to_ints


class NavigationSimulator(ABC):
    """
    Abstract base class for navigation simulators.
    
    Provides common interface and shared rendering functionality.
    Subclasses must implement run_step().
    """
    
    def __init__(self, width: int = 800, height: int = 600):
        self.width = width
        self.height = height
        self.true_pos = np.array([width / 2, height / 2], dtype=float)
        self.img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Sensor noise parameters (set by subclasses)
        self.gps_freq = 2.0
        self.gps_noise_std = 30.0
        self.imu_noise_std = 3.0

    @abstractmethod
    def run_step(self) -> Tuple[Optional[np.ndarray], np.ndarray, float]:
        """
        Run one simulation step.
        
        Returns:
            (gps_reading, imu_reading, dt)
            - gps_reading: [px, py] or None if no GPS update
            - imu_reading: [ax, ay]
            - dt: time step
        """
        pass

    def render(self) -> np.ndarray:
        """Render current state with fade trail."""
        self.img = (self.img * 0.95).astype(np.uint8)
        cv2.circle(self.img, floats_to_ints(self.true_pos), 5, (0, 255, 0), -1)
        return self.img
