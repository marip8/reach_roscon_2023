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

        :param bin_dims: Dimensions of the bin (m)
        :param n_samples: Number of pose samples to generate
        :param bin_normal_deviation: Angular deviation of any output pose z-axis from the unit z-axis (radians)
        :param bin_frame: TF frame associated with the bottom center of the bin
        :param kinematic_base_frame: TF frame associated with the base link of the kinematic chain of the robot
        :param node: ROS2 node
        :param rand_seed: seed number for the random number generator
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

        # TODO: Look up the transform from the robot kinematic base frame to the bin corner frame
        # base_to_bin =

        # Convert to Numpy class member
        self.base_to_bin = to_numpy(base_to_bin)


    def generate(self) -> List[np.ndarray]:
        # Create an array of target poses (n x 4 x 4)
        poses = np.empty((self.n_samples, 4, 4))
        poses[:, :, :] = np.eye(4)

        # TODO: Use numpy to randomly sample target 3D points inside the bin volume. The output array shape should be (n, 3)
        # random_points =

        # Set the translation of the poses to the random points, centered inside the bin volume
        poses[:, :3, 3] = random_points - np.hstack([self.bin_dims[:2] / 2.0, 0])

        # Create a random surface normal
        theta = np.random.random((self.n_samples,)) * 2 * np.pi
        rot_z = create_z_axis_rotations(theta)

        # TODO: Use numpy to generate a set of random surface normal offset angles between [0, bin normal deviation]. The output array shape should be (n,)
        # phi =
        rot_x = create_x_axis_rotations(phi)

        # Set the orientation of the poses
        poses[:, :3, :3] = rot_z @ rot_x

        # Transform the poses into the robot base frame
        poses = self.base_to_bin @ poses

        return [np.squeeze(p) for p in np.split(poses, len(poses))]