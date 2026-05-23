from setuptools import setup

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=[
        'rospy',
        'sensor_msgs',
        'geometry_msgs',
        'opencv-python',
        'numpy',
    ],
    entry_points={
        'console_scripts': [
            f'{package_name}_node = {package_name}.person_follower_node:main',
        ],
    },
)