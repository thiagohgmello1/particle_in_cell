import cv2
import numpy as np
from geometry.topology import Topology


class PrimitiveGeometries:
    def __init__(self, topology: Topology):
        self.topology = topology

    def line(self, init_point: np.array, end_point: np.array, material, thickness=1):
        cv2.line(self.topology.geometry, init_point, end_point, color=material.color, thickness=thickness)
        return np.array([init_point, end_point])


    def circle(self, center: np.array, radius, material, accuracy=10):
        center = self.coordinate_to_matrix(center)
        circle_points = cv2.ellipse2Poly(center, (radius, radius), 0, 0, 360, accuracy)
        self.filled_convex_polygon(circle_points, material.color)
        return circle_points


    def ellipse(self, center: np.array, axes, angle, material, accuracy=10):
        center = self.coordinate_to_matrix(center)
        ellipse_points = cv2.ellipse2Poly(center, axes, angle, 0, 360, accuracy)
        self.filled_convex_polygon(ellipse_points, material.color)
        return ellipse_points


    def filled_convex_polygon(self, points: list, material):
        points = [self.coordinate_to_matrix(point) for point in points]
        cv2.fillConvexPoly(self.topology.geometry, points=points, color=material.color)


    def filled_non_convex_polygon(self, points: list, material):
        points = [self.coordinate_to_matrix(point) for point in points]
        cv2.fillPoly(self.topology.geometry, pts=points, color=material.color)


    def coordinate_to_matrix(self, point: np.array):
        return np.multiply(point, self.topology.convert_factor).astype(int)

