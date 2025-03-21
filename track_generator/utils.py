import numpy as np
from enum import Enum
import yaml


class Mode(Enum):
    """
    Possible modes for how Voronoi regions are selected.

    1. Expand:
        Find closest nodes around starting node.
        Results in roundish track shapes.

    2. Extend:
        Find nodes closest to line extending from starting node.
        Results in elongated track shapes.

    3. Random:
        Select all regions randomly.
        Results in large track shapes.
    """

    EXPAND = 1
    EXTEND = 2
    RANDOM = 3


def closest_node(node, nodes, k):
    """
    Returns the index of the k-th closest node.

    Args:
        node (numpy.ndarray): Node to find k-th closest node to.
        nodes (numpy.ndarray): Available nodes.
        k (int): Number which determines which closest node to return.

    Returns:
        int: Index of k-th closest node.
    """
    deltas = nodes - node
    distance = np.einsum("ij,ij->i", deltas, deltas)
    return np.argpartition(distance, k)[k]


def clockwise_sort(p):
    """
    Sorts nodes in clockwise order.

    Args:
        p (numpy.ndarray): Points to sort.

    Returns:
        numpy.ndarray: Clockwise sorted points.
    """
    d = p - np.mean(p, axis=0)
    s = np.arctan2(d[:, 0], d[:, 1])
    return p[np.argsort(s), :]


def curvature(dx_dt, d2x_dt2, dy_dt, d2y_dt2):
    """
    Calculates the curvature along a line.

    Args:
        dx_dt (numpy.ndarray): First derivative of x.
        d2x_dt2 (numpy.ndarray): Second derivative of x.
        dy_dt (numpy.ndarray): First derivative of y.
        d2y_dt2 (numpy.ndarray): Second derivative of y.

    Returns:
        np.ndarray: Curvature along line.
    """
    return (dx_dt**2 + dy_dt**2) ** -1.5 * (dx_dt * d2y_dt2 - dy_dt * d2x_dt2)


def arc_length(x, y, R):
    """
    Calculates the arc length between to points based on the radius of curvature of the path segment.

    Args:
        x (numpy.ndarray): X-coordinates.
        y (numpy.ndarray): Y-coordinates.
        R (numpy.ndarray): Radius of curvature of track segment in meters.
    Returns:
        (float): Arc length in meters.
    """
    x0, x1 = x[:-1], x[1:]
    y0, y1 = y[:-1], y[1:]
    R = R[:-1]

    distance = np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    theta = 2 * np.arcsin(0.5 * distance / R)
    arc_length = R * theta
    return arc_length


def transformation_matrix(displacement, angle):
    """
    Translate, then rotate around origin.

    Args:
        displacement (tuple): Distance to translate along both axes.
        angle (float): Angle in radians to rotate.

    Returns:
        numpy.ndarray: 3x3 transformation matrix.
    """
    h, k = displacement
    c, s = np.cos(angle), np.sin(angle)

    M = np.array([[c, -s, h * c - k * s], [s, c, h * s + k * c], [0, 0, 1]])
    return M


def load_yaml_config(file_path):
    """
    Load a YAML file into a Python dictionary.

    :param file_path: Path to the YAML file.
    :return: Dictionary representation of the YAML file.
    """
    try:
        with open(file_path, "r") as file:
            data = yaml.safe_load(file)
        return data
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
        return None
    except yaml.YAMLError as e:
        print(f"Error parsing YAML: {e}")
        return None


def parse_config_with_mode(file_path):
    """
    Load a YAML file and parse the mode as an Enum.

    :param file_path: Path to the YAML file.
    :return: Parsed configuration with mode as a Mode enum.
    """
    config = load_yaml_config(file_path)
    if config is None:
        return None

    # Parse mode under 'track' key and convert to Mode enum
    try:
        if "track" in config and "mode" in config["track"]:
            config["track"]["mode"] = Mode[config["track"]["mode"].upper()]
        else:
            raise ValueError("Mode is not defined in the YAML file under 'track'.")
    except KeyError as e:
        print(f"Error: Invalid mode value in YAML. {e}")
        return None
    return config
