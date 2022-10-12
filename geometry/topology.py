import numpy as np
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
            unit: str = 'nm'
    ):
        self.box_size = np.array([x_box, y_box])
        self.resolution = np.array([x_resolution, y_resolution])
        self.unit = self.convert_unit(unit)
        self.geometry = np.zeros((x_resolution, y_resolution, 1), dtype="uint8")
        self.convert_factor = np.divide(self.resolution, self.box_size)
        self.drawer = PrimitiveGeometries(self)
        self.particles = particles
        self.set_particles_topology()


    def set_particles_topology(self):
        if self.particles is not None:
            [particle.set_topology(self) for particle in self.particles]


    def insert_particles(self, material, style: str):
        if style == 'line':
            [self.drawer.poly_lines(particle.positions, material) for particle in self.particles]
        else:
            [self.drawer.arrow(particle.positions, material) for particle in self.particles]


    @staticmethod
    def convert_unit(unit: str):
        units_dict = {'m': 1, 'mm': 1e-3, 'um': 1e-6, 'nm': 1e-9, 'pm': 1e-12}
        return units_dict.get(unit, 1e-6)
