from geometry.topology import Topology
from graphics.window import Window
from materials.material import Material
from particles.particle import Particle
import numpy as np

if __name__ == '__main__':
    particle = Particle(1, 1, np.array([14, 6]), np.array([13, -123]))
    topology = Topology(particles=[particle])
    window = Window(topology)
    a = [1, 1]
    b = [1, 6]
    c = [4, 6]
    d = [4, 1]
    e = [6, 4]
    f = [6, 3]
    g = [6, 6]
    h = [6, 1]
    i = [10, 1]
    j = [10, 6]
    contours = np.array([a, b, c, e, g, j, i, h, f, d])
    material = Material(10, color=255)
    material_draw = Material(1)
    # topology.drawer.circle(np.array([10, 10]), 5, material)
    # topology.drawer.line([0, 0], [10, 10], material)
    # topology.drawer.filled_polygon([contours], material=material)
    particle.move(delta_t=1)
    topology.insert_particles(material, 'arrow')
    window.create_window('Diode')
