from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/ublox_sara_g3_node.py'],)
setup(**d)

