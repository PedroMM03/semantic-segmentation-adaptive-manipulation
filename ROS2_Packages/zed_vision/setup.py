from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'zed_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'msg'),glob('msg/*.msg')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Package for vision processing with ZED camera, segmentation, and object detection using YOLO',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_zed_vision_node = zed_vision.yolo_zed_vision_node:main'
        ],
    },
)