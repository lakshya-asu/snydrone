from setuptools import find_packages, setup

package_name = 'snydrone_brain'

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
            'shot_planner_node = snydrone_brain.shot_planner_node:main',
            'shot_executor_node = snydrone_brain.shot_executor_node:main',
            'llm_planner_node = snydrone_brain.llm_planner_node:main',
        ],
    },
)
