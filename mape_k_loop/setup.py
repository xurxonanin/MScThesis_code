from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mape_k_loop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # Install launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='The mape_k_loop package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mape_k_loop = mape_k_loop.mape_k_loop:main',
            'monitor = mape_k_loop.monitor:main',
            'analyze = mape_k_loop.analyze:main',
            'planning = mape_k_loop.planning:main',
        ],
    },
)
