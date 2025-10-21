from setuptools import find_packages, setup

package_name = 'frisbee_detector'

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
    maintainer='takumi',
    maintainer_email='aquarius1496@gmail.com',
    description='Rule-based detection of a flying frisbee from LiDAR scan data.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frisbee_detector_node = frisbee_detector.frisbee_detector_node:main',
        ],
    },
)
