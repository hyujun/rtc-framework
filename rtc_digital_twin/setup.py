import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rtc_digital_twin'

setup(
    name=package_name,
    version='5.17.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junho Park',
    maintainer_email='jeryblueput@gmail.com',
    description='Generalized Digital Twin RViz2 visualization for any URDF robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'digital_twin_node = rtc_digital_twin.digital_twin_node:main',
        ],
    },
)
