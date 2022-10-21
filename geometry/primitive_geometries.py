import cv2
import numpy as np
from materials.material import Material
from utils.converters import vec_real_to_matrix, scalar_real_to_matrix


class PrimitiveGeometries:
    def __init__(self, topology):
        self.topology = topology

    def line(self, init_point: np.array, end_point: np.array, material: Material, thickness=1, geometry=None):
        init_point = vec_real_to_matrix(self.topology.convert_factor, init_point)
        end_point = vec_real_to_matrix(self.topology.convert_factor, end_point)
        cv2.line(self.topology.geometry, init_point, end_point, color=material.color, thickness=thickness)
        cv2.line(geometry, init_point, end_point, color=material.color, thickness=thickness)
        return np.array([init_point, end_point])


    def poly_lines(self, points: list, material: Material, thickness=1):
        for count in range(len(points) - 1):
            cv2.line(
                self.topology.geometry,
                points[count],
                points[count + 1],
                color=material.color,
                thickness=thickness
            )


    def arrow(self, points: list, material: Material, thickness=1, tip_length=5):
        for count in range(len(points) - 1):
            init_point = points[count]
            end_point = points[count + 1]
            arrow_size = np.linalg.norm(end_point - init_point)
            tip = tip_length / arrow_size
            cv2.arrowedLine(
                self.topology.geometry,
                init_point,
                end_point,
                color=material.color,
                thickness=thickness,
                tipLength=tip
            )


    def circle(self, center: np.array, radius: float, material: Material, accuracy=10):
        center = vec_real_to_matrix(self.topology.convert_factor, center)
        radius = scalar_real_to_matrix(self.topology.convert_factor, radius)
        circle_points = cv2.ellipse2Poly(center, (radius, radius), 0, 0, 360, accuracy)
        self._filled_convex_polygon(circle_points, material)
        return circle_points


    def ellipse(self, center: np.array, axes, angle, material: Material, accuracy=10):
        center = vec_real_to_matrix(self.topology.convert_factor, center)
        ellipse_points = cv2.ellipse2Poly(center, axes, angle, 0, 360, accuracy)
        self._filled_convex_polygon(ellipse_points, material)
        return ellipse_points


    def rectangle(self, init_point: np.array, end_point: np.array, color):
        cv2.rectangle(self.topology.geometry, init_point, end_point, color=color, thickness=1)


    def filled_polygon(self, points: list, material: Material):
        self.topology.materials[material.color] = material
        points = [vec_real_to_matrix(self.topology.convert_factor, point) for point in points]
        self._filled_non_convex_polygon(points, material)


    def _filled_convex_polygon(self, points: list, material: Material):
        cv2.fillConvexPoly(self.topology.geometry, points=points, color=material.color)


    def _filled_non_convex_polygon(self, points: list, material: Material):
        cv2.fillPoly(self.topology.geometry, pts=points, color=material.color)
