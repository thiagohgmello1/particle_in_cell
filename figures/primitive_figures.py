import cv2
import numpy as np
from figures.image import Image


class PrimitiveFigures:
    def __init__(self, image: Image):
        self.image = image

    def line(self, init_point, end_point, thickness=1, color=(255, 255, 255)):
        cv2.line(self.image.image, init_point, end_point, color=color, thickness=thickness)
        return np.array([init_point, end_point])


    def circle(self, center, radius, color=(255, 255, 255), accuracy=10):
        circle_points = cv2.ellipse2Poly(center, (radius, radius), 0, 0, 360, accuracy)
        self.filled_convex_polygon(circle_points, color)
        return circle_points


    def ellipse(self, center, axes, angle, color=(255, 255, 255), accuracy=10):
        ellipse_points = cv2.ellipse2Poly(center, axes, angle, 0, 360, accuracy)
        self.filled_convex_polygon(ellipse_points, color)
        return ellipse_points


    def filled_convex_polygon(self, points, color=(255, 255, 255)):
        cv2.fillConvexPoly(self.image.image, points=points, color=color)


    def filled_non_convex_polygon(self, points, color=(255, 255, 255)):
        cv2.fillPoly(self.image.image, pts=points, color=color)


    def convert_coordinates(self, points: np.array):
        max_x = np.max(points[:, 0])
        max_y = np.max(points[:, 1])
        points[:, 0] = points[:, 0] * (self.image.convert_factor * self.image.x_size) / max_x
        points[:, 1] = points[:, 1] * (self.image.convert_factor * self.image.y_size) / max_y

        return points

