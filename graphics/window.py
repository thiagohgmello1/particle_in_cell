import cv2
from geometry.topology import Topology


class Window:
    def __init__(self, topology: Topology):
        self.topology = topology


    def create_window(self, window_name):
        cv2.imshow(window_name, self.topology.geometry)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def zoom_control(self):
        zoom_factor = 1
        self.topology.geometry = cv2.resize(self.topology.geometry, None, fx=zoom_factor, fy=zoom_factor)


    def scale_control(self):
        self.topology.geometry = cv2.resize(self.topology.geometry, (0, 0), fx=2, fy=2)

