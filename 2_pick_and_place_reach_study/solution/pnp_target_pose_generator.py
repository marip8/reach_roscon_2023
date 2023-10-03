import reach

import numpy as np
from pyquaternion import Quaternion
import rclpy.node
from tf2_ros import Buffer, TransformListener, Time, Duration, TransformStamped
from typing import List

def to_numpy(msg: TransformStamped) -> np.ndarray:
    """ Converts a ROS2 transform stamped message to a Numpy array
    :return: a 4x4 transformation matrix
    """

    transform = np.eye(4)

    # Translation
    transform[:3, 3] = [
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z,
    ]

    # Rotation
    transform[:3, :3] = Quaternion(
        msg.transform.rotation.w,
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
    ).rotation_matrix

    return transform


def create_z_axis_rotations(angles: np.ndarray):
    """ Creates an array of rotation matrices that have been rotated around the z-axis by the input angles

    :param angles: 1-D array of rotation angles (radians)
    :return: an array of size (len(angles), 3, 3), representing an array of z-axis rotation matrices
    """
    rot_z = np.zeros((len(angles), 3, 3))
    rot_z[:, 2, 2] = 1.0
    rot_z[:, 0, 0] = np.cos(angles)
    rot_z[:, 0, 1] = -np.sin(angles)
    rot_z[:, 1, 0] = np.sin(angles)
    rot_z[:, 1, 1] = np.cos(angles)
    return rot_z


def create_x_axis_rotations(angles: np.ndarray):
    """ Creates an array of rotation matrices that have been rotated around the x-axis by the input angles

    :param angles: 1-D array of rotation angles (radians)
    :return: an array of size (len(angles), 3, 3), representing an array of x-axis rotation matrices
    """
    rot_x = np.zeros((len(angles), 3, 3))
    rot_x[:, 0, 0] = 1.0
    rot_x[:, 1, 1] = np.cos(angles)
    rot_x[:, 1, 2] = -np.sin(angles)
    rot_x[:, 2, 1] = np.sin(angles)
    rot_x[:, 2, 2] = np.cos(angles)
    return rot_x


class PnPTargetPoseGenerator(reach.TargetPoseGenerator):
    def __init__(self,
                 bin_dims: List[float],
                 n_samples: int,
                 bin_normal_deviation: float,
                 bin_frame: str,
                 kinematic_base_frame: str,
                 node: rclpy.node.Node,
                 rand_seed: int = 0):
        """

        :param bin_dims:
        :param n_samples:
        :param bin_normal_deviation:
        :param bin_frame: TF frame associated with the bottom center of the bin
        :param kinematic_base_frame:
        :param node:
        :param rand_seed:
        """
        super().__init__()
        # Set the seed for the RNG
        np.random.seed(rand_seed)

        # Save parameters internally
        assert len(bin_dims) == 3
        self.bin_dims = np.array(bin_dims)
        self.n_samples = n_samples
        self.bin_normal_deviation = bin_normal_deviation

        # Create a transform listener
        buffer = Buffer()
        _ = TransformListener(buffer, node)
        rclpy.spin_once(node)

        # Look up the transform from the robot kinematic base frame to the bin corner frame
        base_to_bin = buffer.lookup_transform(kinematic_base_frame, bin_frame, Time(), Duration(seconds=3))

        # Convert to Numpy class member
        self.base_to_bin = to_numpy(base_to_bin)


    def generate(self) -> List[np.ndarray]:
        # Create an array of target poses (n x 4 x 4)
        poses = np.empty((self.n_samples, 4, 4))
        poses[:, :, :] = np.eye(4)

        # Randomly sample target 3D points inside the bin volume
        random_points = np.random.random((self.n_samples, 3)) * self.bin_dims
        poses[:, :3, 3] = random_points - np.hstack([self.bin_dims[:2] / 2.0, 0])

        # Create a random surface normal
        theta = np.random.random((self.n_samples,)) * 2 * np.pi
        rot_z = create_z_axis_rotations(theta)

        phi = np.random.random((self.n_samples,)) * self.bin_normal_deviation
        rot_x = create_x_axis_rotations(phi)

        poses[:, :3, :3] = rot_z @ rot_x

        # Transform the poses into the robot base frame
        poses = self.base_to_bin @ poses

        return [np.squeeze(p) for p in np.split(poses, len(poses))]