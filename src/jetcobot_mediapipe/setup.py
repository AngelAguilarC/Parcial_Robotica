from setuptools import find_packages, setup

package_name = 'jetcobot_mediapipe'

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
            '01_HandDetector = jetcobot_mediapipe.01_HandDetector:main',
            '02_PoseDetector = jetcobot_mediapipe.02_PoseDetector:main',
            '03_Holistic = jetcobot_mediapipe.03_Holistic:main',
            '04_FaceMesh = jetcobot_mediapipe.04_FaceMesh:main',
            '05_FaceDetection = jetcobot_mediapipe.05_FaceDetection:main',
            '06_FaceLandmarks = jetcobot_mediapipe.06_FaceLandmarks:main',
            '07_Objectron = jetcobot_mediapipe.07_Objectron:main',
            '08_VirtualPaint = jetcobot_mediapipe.08_VirtualPaint:main',
            '09_HandCtrl = jetcobot_mediapipe.09_HandCtrl:main',
            '10_GestureRecognition = jetcobot_mediapipe.10_GestureRecognition:main',
            'FingerTrajectory = jetcobot_mediapipe.FingerTrajectory:main',
            'FingerCtrl = jetcobot_mediapipe.FingerCtrl:main',
            'PoseArm = jetcobot_mediapipe.PoseArm:main',
            'Find_Hand = jetcobot_mediapipe.Find_Hand:main',
            'Hand_Ctrl = jetcobot_mediapipe.Hand_Ctrl:main',
        ],
    },
)
