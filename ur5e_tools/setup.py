from setuptools import find_packages, setup

package_name = 'ur5e_tools'

setup(
    name=package_name,
    version='5.2.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Python development utilities for the UR5e RT controller stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_gui = ur5e_tools.gui.controller_gui:main',
            'motion_editor_gui = ur5e_tools.gui.motion_editor_gui:main',
            'monitor_data_health = ur5e_tools.monitoring.monitor_data_health:main',
            'plot_ur_trajectory = ur5e_tools.plotting.plot_ur_trajectory:main',
            'hand_udp_sender_example = ur5e_tools.utils.hand_udp_sender_example:main',
            'compare_mjcf_urdf = ur5e_tools.validation.compare_mjcf_urdf:main'
        ],
    },
)
