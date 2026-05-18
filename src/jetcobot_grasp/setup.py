from setuptools import find_packages, setup

package_name = 'jetcobot_grasp'

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
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        '1_gesture_recognition_stacking = jetcobot_grasp.1_gesture_recognition_stacking:main',
        '2_color_recognition_grasp = jetcobot_grasp.2_color_recognition_grasp:main',
        '3_apriltag_recognition_grasp = jetcobot_grasp.3_apriltag_recognition_grasp:main',
        '4_put_and_grasp = jetcobot_grasp.4_put_and_grasp:main',
        ],
    },
)
