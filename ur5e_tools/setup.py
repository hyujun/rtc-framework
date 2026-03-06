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
        ('lib/' + package_name, [
            'scripts/monitor_data_health.py',
            'scripts/plot_ur_trajectory.py',
            'scripts/motion_editor_gui.py',
            'scripts/hand_udp_sender_example.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Python development utilities for the UR5e RT controller stack.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
