from setuptools import setup
import os
from glob import glob
package_name = 'image_processing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    py_modules=[
        'image_processing.utils'
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tbs group',
    maintainer_email='wmm@todo.todo',
    description='Processes depth, rgb, and ir images taken from ROS topics',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mask_display = image_processing.mask_display_node:main',
            'image_display = image_processing.image_display_node:main',
            'image_processor = image_processing.image_processor_node:main',
            'image_saver = image_processing.image_save_node:main',
            'ranger_control_demo = image_processing.ranger_control_demo:main',
            'zed_depth_test = image_processing.zed_depth_test:main'
        ],
    },
)
