import cv2
import numpy as np
import matplotlib.pyplot as plt
from particles.particle import Particle
from geometry.primitive_geometries import PrimitiveGeometries


class Topology:
    def __init__(
            self,
            x_box: float = 20,
            y_box: float = 20,
            x_resolution: int = 400,
            y_resolution: int = 400,
            particles: list[Particle] = None,
            scale: str = 'n'
    ):
        self.box_size = np.array([x_box, y_box])
        self.resolution = np.array([x_resolution, y_resolution])
        self.unit = self.convert_unit(scale)
        self.geometry = np.ones((x_resolution, y_resolution), dtype="uint8")
        self.edges = np.zeros((x_resolution, y_resolution), dtype="uint8")
        self.convert_factor = np.divide(self.resolution, self.box_size)
        self.drawer = PrimitiveGeometries(self)
        size = np.multiply(self.box_size, self.convert_factor).astype(int) - 1
        self.drawer.rectangle(np.array([0, 0]), size, (255, 255, 255))
        self.particles = particles
        self.set_particles_topology()


    def set_particles_topology(self):
        if self.particles is not None:
            [particle.set_topology(self) for particle in self.particles]


    def insert_particles(self, material, style: str):
        if style == 'line':
            [
                self.drawer.poly_lines(particle.positions, material, thickness=particle.size)
                for particle in self.particles
            ]
        else:
            [
                self.drawer.arrow(particle.positions, material, thickness=particle.size)
                for particle in self.particles
            ]


    def get_edges(self):
        ret, thresh_img = cv2.threshold(self.geometry, 150, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(
            image=thresh_img, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE
        )
        cv2.drawContours(self.edges, contours, -1, (255, 255, 255), 1)

    @staticmethod
    def convert_unit(unit: str):
        units_dict = {'1': 1, 'm': 1e-3, 'u': 1e-6, 'n': 1e-9, 'p': 1e-12}
        return units_dict.get(unit, 1e-9)
