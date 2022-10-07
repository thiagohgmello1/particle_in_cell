import cv2
from figures.image import Image


class Window:
    def __init__(self, image: Image):
        self.image = image


    def create_window(self, window_name):
        cv2.imshow(window_name, self.image.image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    def zoom_control(self):
        scale = 150
        x_size = int(self.image.x_size * scale / 100)
        y_size = int(self.image.y_size * scale / 100)
        dimension = (x_size, y_size)
        self.image.image = cv2.resize(self.image.image, dimension,
                                      interpolation=cv2.INTER_AREA)

