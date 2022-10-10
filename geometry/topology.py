import numpy as np


class Topology:
    def __init__(self, x_box=20, y_box=20, x_resolution=400, y_resolution=400, unit='nm'):
        self.box_size = np.array([x_box, y_box])
        self.resolution = np.array([x_resolution, y_resolution])
        self.unit = self.convert_unit(unit)
        self.geometry = np.zeros((x_resolution, y_resolution, 1), dtype="uint8")
        self.convert_factor = np.divide(self.resolution, self.box_size)


    @staticmethod
    def convert_unit(unit: str):
        units_dict = {'m': 1, 'mm': 1e-3, 'um': 1e-6, 'nm': 1e-9, 'pm': 1e-12}
        return units_dict.get(unit, 1e-6)
