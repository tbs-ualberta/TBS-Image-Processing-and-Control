from setuptools import setup

package_name = 'image_processing'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wmm',
    maintainer_email='wmm@todo.todo',
    description='Processes depth, rgb, and ir images taken from ROS topics',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mask_display = image_processing.mask_display:main',
        ],
    },
)
