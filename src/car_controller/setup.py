from setuptools import find_packages, setup

package_name = 'car_controller'

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
    maintainer='tu',
    maintainer_email='tu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop_cmd_vel = car_controller.teleop_cmd_vel:main',
            'test_nav2_only = car_controller.test_nav2_only:main',
            'testdocking = car_controller.testdocking:main',
            'dockinglive = car_controller.dockinglive:main',
            'master_brain = car_controller.master_brain:main',
        ],
    },
)
