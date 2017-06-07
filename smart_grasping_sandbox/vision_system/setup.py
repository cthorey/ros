from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['vision_system'],
    package_dir={'': 'src'},
    requires=['cv_bridge', 'rospy', 'sensor_msgs', 'std_msgs'])

setup(**setup_args)
