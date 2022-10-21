from geometry.topology import Topology
from graphics.window import Window
from materials.material import Material
from particles.particle import Particle
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    particle = Particle(1, 1)
    topology = Topology(particles=[particle], x_resolution=400, y_resolution=400)
    #                                                      x   y               vx   vy
    topology.particles[0].set_initial_conditions(np.array([1, 5]), np.array([0, 10]))
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
    material = Material(10, color=60)
    # a = [1, 1]
    # b = [1, 6]
    # c = [6, 6]
    # d = [6, 1]
    # contours = np.array([a, b, c, d])
    material_draw = Material(1, color=255)
    # topology.drawer.circle(np.array([4, 6]), 3, material)
    # topology.drawer.line([1, 1], [1, 18], material)
    # topology.drawer.line([3, 1], [3, 16], material)
    topology.drawer.filled_polygon([contours], material=material)
    topology.get_edges()
    particle.move(delta_t=1)
    topology.insert_particles(material_draw, 'arrow')
    print(particle.positions / topology.convert_factor)
    print(particle.velocities / topology.convert_factor)
    window.create_window('Diode')
