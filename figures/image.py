import numpy as np


class Image:
    def __init__(self, x_size=400, y_size=400, convert_factor=0.8):
        self.x_size = x_size
        self.y_size = y_size
        self.convert_factor = convert_factor
        self.image = np.zeros((self.x_size, self.y_size, 1), dtype="uint8")
