from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['structured_testing'],
    package_dir={'': 'src', 
                '': 'src/Evaluators'}
)

setup(**d)