from setuptools import find_packages, setup

package_name = 'rtc_tools'

setup(
    name=package_name,
    version='5.17.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junho Park',
    maintainer_email='jeryblueput@gmail.com',
    description='Python development utilities for the RTC (Real-Time Controller) framework.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot_rtc_log = rtc_tools.plotting.plot_rtc_log:main',
            'plot_ur_log = rtc_tools.plotting.plot_rtc_log:main',
            'plot_ur_trajectory = rtc_tools.plotting.plot_rtc_log:main',
            'hand_udp_sender_example = rtc_tools.utils.hand_udp_sender_example:main',
            'compare_mjcf_urdf = rtc_tools.validation.compare_mjcf_urdf:main',
            'urdf_to_mjcf = rtc_tools.conversion.urdf_to_mjcf:main'
        ],
    },
)
