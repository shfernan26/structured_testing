from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['structured_testing', 'structured_testing/observers'],
    scripts=['src/start_sim.py'],
    package_dir={'': 'src'}
)

setup(**d)