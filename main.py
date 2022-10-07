from figures.image import Image
from figures.primitive_figures import PrimitiveFigures
from graphics.window import Window
import numpy as np

if __name__ == '__main__':
    image = Image()
    window = Window(image)
    primitive_figures = PrimitiveFigures(image)
    a = [100, 200]
    b = [200, 200]
    c = [100, 100]
    d = [200, 100]
    e = [250, 160]
    f = [250, 140]
    g = [250, 200]
    h = [250, 100]
    i = [350, 200]
    j = [350, 100]
    contours = np.array([a, b, e, g, i, j, h, f, d, c])
    # primitive_figures.line((50, 50), (100, 50))
    primitive_figures.circle((200, 200), 50)
    # primitive_figures.filled_non_convex_polygon([contours])
    window.zoom_control()
    window.create_window('Diode')
