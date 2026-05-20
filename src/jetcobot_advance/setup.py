from setuptools import setup
import os
from glob import glob
package_name = 'jetcobot_advance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='yahboom@example.com',
    description='Finger trajectory tracking package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_tracking = jetcobot_advance.apriltag_tracking:main',
            'kcf_tracking = jetcobot_advance.kcf_tracking:main',
        ],
    },
)