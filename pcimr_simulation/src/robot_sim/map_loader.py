# Third party modules
import numpy as np
from PIL import Image


def load_map(image_path: str) -> np.ndarray:
    """
    Get a numpy array of an image so that one can access values[x][y].

    @param image_path: The path to the image to be loaded.
    @return: A numpy array containing the map
    """
    image = Image.open(image_path, "r")
    width, height = image.size
    pixel_values = list(image.getdata())
    if image.mode == "RGB":
        channels = 3
    elif image.mode == "L":
        channels = 1
    else:
        print("Unknown mode: %s" % image.mode)
        return None
    pixel_values = np.array(pixel_values).reshape((width, height, channels))

    pixel_values = np.rot90(pixel_values, 3)
    pixel_values = 255 - pixel_values
    pixel_values[pixel_values == 81] = -1

    return np.squeeze(pixel_values, axis=2)


def print_map(image):
    symbols = ['O', 'X']

    for y in range(image.shape[1] - 1, -1, -1):
        line = ''
        for x in range(image.shape[0]):
            line += symbols[int(image[x, y])] + ' '
        line += ''
        print('{}: {}'.format(y, line))
