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
        self.velocities = []


    def set_topology(self, topology):
        self.topology = topology
        self.box = np.zeros(self.topology.geometry.shape)


    def external_set_initial_conditions(self, position, velocity):
        self.position = position
        self.velocity = velocity
        self._convert_particle_to_matrix()


    def set_initial_conditions(self, electric_field):
        self.set_initial_position()
        self.set_initial_velocity(electric_field)


    def set_initial_position(self):
        max_random_number = self.topology.resolution[0]
        random_init_pos = np.random.randint(max_random_number, size=2)
        material = self.topology.geometry[tuple(random_init_pos)]
        while material <= 1:
            random_init_pos = np.random.randint(max_random_number, size=2)
            material = self.topology.geometry[tuple(random_init_pos)]
        self.position = random_init_pos
        self.positions.append(self.position)


    def set_initial_velocity(self, electric_field):
        material = self.topology.materials[self.topology.geometry[tuple(self.position)]]
        relax_time = material.relax_time
        electron_mass = material.electron_mass
        fermi_velocity = material.fermi_velocity
        velocity = fermi_velocity + material.electron_charge * electric_field * relax_time/ electron_mass
        return velocity


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
        self.velocities.append(velocity)
        if not collision:
            return 0, end_point, velocity, False
        delta_t, end_point, velocity = self.calc_new_parameters(
            delta_t, init_point, collision_point, velocity, normal_vector
        )
        return delta_t, end_point, velocity, True


    def find_collision_point(self, init_point, end_point, velocity):
        edge_points = self.topology.edge_points
        box = self.box.copy()
        cv2.line(box, init_point, end_point, color=(1, 1, 1), thickness=1)
        internal_path = np.multiply(box, self.topology.geometry)
        external_path = np.multiply(box, self.topology.inv_geometry)
        internal_path = (internal_path / np.amax(internal_path, initial=1)).astype(int)
        external_path = (external_path / np.amax(external_path, initial=1)).astype(int)
        contact_points = np.flip(np.transpose(np.nonzero(external_path)), 1)
        if contact_points.size == 0:
            return False, None, None
        contact_points = np.array([contact_points[0], contact_points[-1]])
        distance_from_init_point = np.linalg.norm(contact_points - init_point, axis=1)
        external_collision_point = contact_points[np.argmin(distance_from_init_point)]
        collision_point = self.get_collision_point(internal_path, external_collision_point, init_point)
        normal_vector = self.get_normal_vector(edge_points, collision_point, velocity)
        return True, normal_vector, collision_point


    def get_normal_vector(self, edge_points, collision_point, velocity):
        closer_points = self.get_closer_points(edge_points, collision_point)
        parallel_vector = closer_points[0] - closer_points[1]
        normal_vector = np.rint(np.matmul(rotate_matrix(90), parallel_vector))
        if np.dot(normal_vector, velocity) > 0:
            normal_vector = np.rint(np.matmul(rotate_matrix(180), normal_vector))
        return normal_vector.astype(int)


    @staticmethod
    def get_collision_point(internal_path, external_collision_point, init_point):
        external_collision_point = np.flip(external_collision_point)
        internal_path = np.transpose(np.nonzero(internal_path))
        dist, closer_points_index = spatial.KDTree(internal_path).query(external_collision_point, k=2)
        if dist[0] == dist[1]:
            closer_points = internal_path[closer_points_index]
            desired_point = closer_points[spatial.KDTree(closer_points).query(init_point)[1]]
            return np.flip(desired_point)
        else:
            closer_point_index = closer_points_index[np.argmin(dist)]
            desired_point = internal_path[closer_point_index]
            return np.flip(desired_point)


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
