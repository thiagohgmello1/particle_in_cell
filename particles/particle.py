import cv2
import numpy as np
from utils.converters import vec_real_to_matrix, scalar_real_to_matrix


class Particle:
    def __init__(
            self,
            mass: float,
            charge: float,
            init_pos: np.array,
            init_velocity: np.array,
            size: float = 0,
            density: float = 1
    ):
        self.positions = []
        self.position = init_pos
        self.velocity = init_velocity
        self.mass = mass
        self.charge = charge
        self.size = size
        self.density = density
        self.topology = None
        self.box = None


    def set_topology(self, topology):
        self.topology = topology
        self.box = np.zeros(self.topology.geometry.shape)
        self._convert_particle_to_matrix()


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
        d_t, pos, vel, coll_bdy = self.boundary_collision(delta_t, position, velocity)
        d_t, pos, vel, coll_tly = self.topology_collision(delta_t, position, pos, velocity)
        return d_t, pos, vel, coll_tly


    def topology_collision(self, delta_t, init_point, end_point, velocity):
        # points = np.array([init_point, end_point])
        cv2.line(self.box, init_point, end_point, color=(1, 1, 1), thickness=1)
        # aux_box = self.box[
        #           np.min(points[:, 1]):np.max(points[:, 1])+1, np.min(points[:, 0]):np.max(points[:, 0])+1
        #           ]
        # edges = self.topology.edges[
        #         np.min(points[:, 1]):np.max(points[:, 1])+1, np.min(points[:, 0]):np.max(points[:, 0])+1
        #         ]
        edges = np.multiply(self.box, self.topology.edges)
        edges = edges / np.amax(edges)
        contact_points = np.transpose(np.nonzero(edges))
        if contact_points.size == 0:
            return delta_t, end_point, velocity, False
        return delta_t, end_point, velocity, False


    def boundary_collision(self, delta_t, position, velocity):
        collision = False
        new_position = (position + velocity * delta_t).astype(int)
        new_velocity = np.copy(velocity)
        new_delta_t = delta_t
        if new_position[0] > self.topology.resolution[0]:
            normal_versor = np.array([-1, 0])
            collision = True
            new_delta_t = (self.topology.resolution[0] - position[0]) / velocity[0]
            new_velocity = velocity - 2 * (np.dot(velocity, normal_versor)) * normal_versor
        elif new_position[0] < 0:
            normal_versor = np.array([1, 0])
            collision = True
            new_delta_t = -position[0] / velocity[0]
            new_velocity = velocity - 2 * (np.dot(velocity, normal_versor)) * normal_versor
        if new_position[1] > self.topology.resolution[1] and \
                new_delta_t >= (self.topology.resolution[1] - position[1]) / velocity[1]:
            normal_versor = np.array([0, -1])
            collision = True
            if new_delta_t > (self.topology.resolution[1] - position[1]) / velocity[1]:
                new_velocity = velocity - 2 * (np.dot(velocity, normal_versor)) * normal_versor
            else:
                new_velocity[1] = (velocity - 2 * (np.dot(velocity, normal_versor)) * normal_versor)[1]
            new_delta_t = (self.topology.resolution[1] - position[1]) / velocity[1]
        elif new_position[1] < 0 and new_delta_t >= -position[1] / velocity[1]:
            normal_versor = np.array([0, 1])
            collision = True
            if new_delta_t > (self.topology.resolution[1] - position[1]) / velocity[1]:
                new_velocity = velocity - 2 * (np.dot(velocity, normal_versor)) * normal_versor
            else:
                new_velocity[1] = (velocity - 2 * (np.dot(velocity, normal_versor)) * normal_versor)[1]
            new_delta_t = -position[1] / velocity[1]

        new_position = (position + velocity * new_delta_t).astype(int)
        delta_t = delta_t - new_delta_t
        self.positions.append(new_position.astype(int))

        return delta_t, new_position, new_velocity, collision
