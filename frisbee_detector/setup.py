from setuptools import setup
from glob import glob
import os

package_name = 'frisbee_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/frisbee_detector.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takumi',
    maintainer_email='example@example.com',
    description='Frisbee detector',
    license='',
    entry_points={
        'console_scripts': [
            'frisbee_detector = frisbee_detector.frisbee_detector_node:main',
        ],
    },
)
