from setuptools import find_packages, setup

package_name = 'jetcobot_visual'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'create_qrcode = jetcobot_visual.create_qrcode:main',
            'parse_qrcode = jetcobot_visual.parse_qrcode:main',
            'detect_pose = jetcobot_visual.detect_pose:main',
            'detect_object = jetcobot_visual.detect_object:main',
            'simple_ar = jetcobot_visual.simple_ar:main',
            'astra_rgb_image = jetcobot_visual.astra_rgb_image:main',
            'astra_depth_image = jetcobot_visual.astra_depth_image:main',
            'pub_image = jetcobot_visual.pub_image:main',
        ],
    },
)
