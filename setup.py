from setuptools import setup

package_name = 'structured_testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=['structured_testing', 'structured_testing/observers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sachin',
    maintainer_email='shfernan@uwaterloo.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'structured_testing = structured_testing.start_sim:main'
        ],
    }
)
