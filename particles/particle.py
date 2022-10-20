import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy import spatial
from sklearn.neighbors import NearestNeighbors
from utils.converters import vec_real_to_matrix, scalar_real_to_matrix, rotate_matrix


class Particle:
    def __init__(self, mass: float, charge: float, size: float = 0):
        self.positions = []
        self.mass = mass
        self.charge = charge
        self.size = size
        self.position = None
        self.velocity = None
        self.topology = None
        self.box = None


    def set_topology(self, topology):
        self.topology = topology
        self.box = np.zeros(self.topology.geometry.shape)


    def set_initial_conditions(self, position, velocity):
        self.position = position
        self.velocity = velocity
        self._convert_particle_to_matrix()


    def _convert_particle_to_matrix(self):
        self._convert_pos_real_to_matrix()
        self._convert_vel_real_to_matrix()
        self._convert_size_real_to_matrix()


    def _convert_pos_real_to_matrix(self):
        self.position = vec_real_to_matrix(self.topology.convert_factor, self.position)
        self.positions.append(self.position)


    def _convert_vel_real_to_matrix(self):
        self.velocity = vec_real_to_matrix(self.topology.convert_factor, self.velocity)


    def _convert_size_real_to_matrix(self):
        self.size = scalar_real_to_matrix(self.topology.convert_factor, self.size)


    def move(self, delta_t: float):
        position = self.position
        velocity = self.velocity
        while True:
            delta_t, position, velocity, collision = self.collision(delta_t, position, velocity)
            if not collision:
                break
        self.position = position
        self.velocity = velocity


    def collision(self, delta_t, position, velocity):
        pos = self.boundary_collision(delta_t, position, velocity)
        d_t, pos, vel, coll_tly = self.topology_collision(delta_t, position, pos, velocity)
        self.positions.append(pos.astype(int))

        return d_t, pos, vel, coll_tly


    def boundary_collision(self, delta_t, position, velocity):
        new_position = (position + velocity * delta_t).astype(int)
        if np.any(np.where(new_position > self.topology.resolution[0])) or np.any(np.where(new_position < 0)):
            delta_t_x_min = -position[0] / velocity[0]
            delta_t_x_max = (self.topology.resolution[0] - position[0]) / velocity[0]
            delta_t_y_min = -position[1] / velocity[1]
            delta_t_y_max = (self.topology.resolution[1] - position[1]) / velocity[1]
            delta_t = [delta_t_x_min, delta_t_x_max, delta_t_y_min, delta_t_y_max]
            new_delta_t = min([dt for dt in delta_t if dt > 0])
            new_position = (position + velocity * new_delta_t).astype(int)
        return new_position


    def topology_collision(self, delta_t, init_point, end_point, velocity):
        collision, normal_vector, collision_point = self.find_collision_point(init_point, end_point, velocity)
        if not collision:
            return 0, end_point, velocity, False
        delta_t, end_point, velocity = self.calc_new_parameters(
            delta_t, init_point, collision_point, velocity, normal_vector
        )
        return delta_t, end_point, velocity, True


    def find_collision_point(self, init_point, end_point, velocity):
        edge_points = np.transpose(np.nonzero(self.topology.edges))
        box = self.box.copy()

        cv2.line(box, init_point, end_point, color=(1, 1, 1), thickness=1)
        edges = np.multiply(box, self.topology.geometry)
        edges = (edges / np.amax(edges, initial=1)).astype(int)
        contact_points = np.flip(np.transpose(np.nonzero(edges)), 1)
        contact_points = contact_points[np.where(np.not_equal(contact_points, init_point).all(1))[0], :]
        if contact_points.size == 0:
            return False, None, None
        contact_points = np.array([contact_points[0], contact_points[-1]])
        distance_from_init_point = np.linalg.norm(contact_points - init_point, axis=1)
        collision_point = contact_points[np.argmin(distance_from_init_point)]
        collision_point = self.get_collision_point(self.topology.edges, init_point, collision_point)
        normal_vector = self.get_normal_vector(edge_points, collision_point, velocity)
        return True, normal_vector, collision_point

    def get_normal_vector(self, edge_points, collision_point, velocity):
        closer_points = self.get_closer_points(edge_points, collision_point)
        normal_vector = np.matmul(rotate_matrix(90), closer_points[0] - closer_points[1]).astype(int)
        if np.dot(normal_vector, velocity) > 0:
            normal_vector = np.matmul(rotate_matrix(180), normal_vector).astype(int)
        return normal_vector


    def get_collision_point(self, box, init_point, collision_point):
        particle_path = np.transpose(np.nonzero(box))
        closer_points = self.get_closer_points(particle_path, collision_point)
        desired_point = closer_points[np.argmin([self.topology.geometry[tuple(pos)] for pos in closer_points])]
        return desired_point


    @staticmethod
    def get_closer_points(geometry, desired_point):
        desired_point = np.flip(desired_point)
        closer_point_index = spatial.KDTree(geometry).query(desired_point)[1]
        neighbors = NearestNeighbors(n_neighbors=3, algorithm='ball_tree').fit(geometry)
        distances, indices = neighbors.kneighbors(geometry)
        return np.flip(geometry[indices[closer_point_index, 1:]], 1)


    @staticmethod
    def calc_new_parameters(delta_t, init_point, collision_point, velocity, normal_vector):
        time_to_collision = np.linalg.norm((collision_point - init_point)) / np.linalg.norm(velocity)
        remaining_time = delta_t - time_to_collision
        new_velocity = velocity - 2 * (np.dot(velocity, normal_vector)) / \
                       (np.linalg.norm(normal_vector) ** 2) * normal_vector

        return remaining_time, collision_point, new_velocity
