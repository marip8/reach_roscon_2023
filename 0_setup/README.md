# Setup
First, clone this repository into a new `colcon` workspace

```commandline
# Make a colcon workspace
mkdir -p colcon_ws/src
cd colcon_ws/src

# Clone the workshop repository
git clone https://github.com/marip8/reach_roscon_2023.git

# Install the python dependencies
python3 -m pip install -r reach_roscon_2023/requirements.txt
```

## Dependency install
Now that you have a workspace for this repository, there are several different ways to install the required dependencies:

1. [Local binary install (recommended)](#local-binary-install)
1. [Build from source](#build-from-source)

### Local binary install
> Note: only ROS2 Humble is supported for this type of install

```commandline
# Install the dependencies
cd colcon_ws
rosdep install --from-paths src -iry

# Build the repository
cd colcon_ws
colcon build --symlink-install 

# Add the install directory of the `reach` Python bindings to the `PYTHONPATH` environment variable
echo "export PYTHONPATH=$PYTHONPATH:/opt/ros/humble/lib/python3/dist-packages" >> ~/.bashrc
```

> Note: The `PYTHONPATH` environment variable must be updated due to small bug in the installation of the `reach` Python bindings in version 1.6.0-1.

### Build from source
| Distro    | Support |
|:----------|:--------|
| Foxy      | &check; |
| Galactic  | ?       |
| Humble    | &check; |
| Iron      | ?       |
| Rolling   | &check; | 

```commandline
# Install the dependencies
cd colcon_ws
vcs import src < src/reach_roscon_2023/dependencies.repos
rosdep install --from-paths src -iry

# Build the repository
colcon build --symlink-install
```
