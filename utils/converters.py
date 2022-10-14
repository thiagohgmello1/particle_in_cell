import numpy as np


def vec_real_to_matrix(converter_factor: np.array, point: np.array):
    return np.multiply(point, converter_factor).astype(int)


def scalar_real_to_matrix(converter_factor: np.array, scalar: float):
    return max(1, scalar * np.mean(converter_factor).astype(int))


def rotate_matrix(theta: float):
    theta = np.deg2rad(theta)
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
