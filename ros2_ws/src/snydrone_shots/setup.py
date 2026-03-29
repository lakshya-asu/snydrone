from setuptools import find_packages, setup

package_name = 'snydrone_shots'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flux',
    maintainer_email='flux@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_pose_node = snydrone_shots.target_pose_node:main',
            'orbit_shot_node = snydrone_shots.orbit_shot_node:main',
            'path_trail_node = snydrone_shots.path_trail_node:main',
        ],
    },
)
