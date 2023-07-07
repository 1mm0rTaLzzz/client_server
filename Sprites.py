import numpy as np
import pygame as pg


class Robot:
    def __init__(self, screen):
        self.transform = np.eye(3)
        self.screen = screen
        self.lWheelPoint = [[0, 0, 1], [10, 0, 1], [10, 10, 1], [0, 10, 1]]

    def draw(self):
        lWheelPoints = self.lWheelPoint @ self.transform
        pg.draw.circle(self.screen, (0, 0, 255), (250, 250), 75)
        pg.draw.aalines(self.screen, (0, 0, 255), True, lWheelPoints)
