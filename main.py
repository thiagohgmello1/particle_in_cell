from geometry.topology import Topology
from graphics.window import Window
from materials.material import Material
from particles.particle import Particle
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    particle = Particle(1, 1)
    topology = Topology(particles=[particle], x_box=3000, y_box=3000, x_resolution=800, y_resolution=800, scale='u')
    #                                                      x   y               vx   vy
    topology.particles[0].external_set_initial_conditions(np.array([1200, 1200]), np.array([-1200, 953]))
    window = Window(topology)
    a = [1000, 1000]
    b = [1000, 2000]
    c = [1500, 2000]
    f = [1960, 1450]
    g = [1960, 1550]
    h = [1500, 1000]
    i = [1960, 2000]
    j = [1960, 1000]
    k = [2460, 1000]
    l = [2460, 2000]
    contours = np.array([a, b, c, g, i, l, k, j, f, h])
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
    particle.move(delta_t=10)
    topology.insert_particles(material_draw, 'arrow')
    # print(particle.positions / topology.convert_factor)
    # print(particle.velocities / topology.convert_factor)
    window.create_window('Diode')
