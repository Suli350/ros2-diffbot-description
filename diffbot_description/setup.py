import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'diffbot_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sulaiman',
    maintainer_email='suliman31991@gmail.com',
    description='URDF/xacro differential drive robot with a kinematic simulator, TF and RViz.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diff_drive_sim = diffbot_description.diff_drive_sim:main',
            'circle_driver = diffbot_description.circle_driver:main',
        ],
    },
)
