from geometry.topology import Topology
from geometry.primitive_geometries import PrimitiveGeometries
from graphics.window import Window
from materials.material import Material
import numpy as np

if __name__ == '__main__':
    topology = Topology()
    window = Window(topology)
    primitive_figures = PrimitiveGeometries(topology)
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
    # primitive_figures.circle((10, 10), 100)
    material = Material(10, color=150)
    primitive_figures.filled_non_convex_polygon([contours], material=material)
    window.create_window('Diode')
