from setuptools import find_packages, setup

package_name = 'drone_image_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
        ['config/siyi_a8_calib.npz']),
        ("share/" + package_name + "/models",
        ["models/yolo26n.pt"]),
        ("share/" + package_name + "/models",
        ["models/targets.pt"]),
    ],
    install_requires=['setuptools', 'ultralytics'],
    zip_safe=True,
    maintainer='Simon Pomerleau',
    maintainer_email='simon@dronolab.local',
    description='ROS 2 YOLO target detection node with camera calibration and unit-vector projection',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "yolo_detector = drone_image_detection.yolo_detector:main",
        ],
    },
)
