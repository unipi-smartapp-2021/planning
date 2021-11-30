from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    scripts=['src/STP/Stp.py'],
    packages=['STP','LTP'],
    package_dir={'': 'src'}
)

setup(**d)