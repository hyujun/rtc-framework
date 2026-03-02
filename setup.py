#!/usr/bin/env python3
"""
setup.py - Python package setup for UR5e RT Controller scripts
"""

from setuptools import setup, find_packages
import os
from pathlib import Path

# Package metadata
NAME = "ur5e_rt_controller"
VERSION = "4.0.0"
DESCRIPTION = "UR5e Real-time controller Python utilities"
AUTHOR = "Your Name"
EMAIL = "your.email@example.com"
URL = "https://github.com/your-repo/ur5e_rt_controller"

# Read requirements
REQUIREMENTS_FILE = Path(__file__).parent / "requirements.txt"
if REQUIREMENTS_FILE.exists():
    with open(REQUIREMENTS_FILE) as f:
        install_requires = [line.strip() for line in f if line.strip() and not line.startswith('#')]
else:
    install_requires = [
        "matplotlib>=3.5.0",
        "pandas>=1.5.0",
        "numpy>=1.21.0",
        "scipy>=1.8.0",
        "rclpy>=0.20.0"
    ]

# Read README
README_PATH = Path(__file__).parent / "README.md"
long_description = README_PATH.read_text(encoding='utf-8') if README_PATH.exists() else DESCRIPTION

setup(
    name=NAME.lower(),
    version=VERSION,
    author=AUTHOR,
    author_email=EMAIL,
    description=DESCRIPTION,
    long_description=long_description,
    long_description_content_type="text/markdown",
    url=URL,
    packages=find_packages(),
    package_dir={'': 'scripts'},
    package_data={
        '': ['*.py', '*.yaml', '*.csv'],
    },
    install_requires=install_requires,
    python_requires=">=3.10",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    entry_points={
        'console_scripts': [
            'monitor_data_health_v2=scripts.monitor_data_health_v2:main',
            'plot_ur_trajectory=scripts.plot_ur_trajectory:main',
            'generate_statistics=scripts.generate_statistics:main',
        ],
    },
    zip_safe=False,
)

# Print installation instructions
print("""
✅ Python package setup complete!

📦 Installed executables:
   - monitor_data_health_v2
   - plot_ur_trajectory       
   - generate_statistics

🔧 Usage examples:
   python3 -m pip install -e .
   
   monitor_data_health_v2 --help
   plot_ur_trajectory /tmp/ur5e_control_log.csv
""")
