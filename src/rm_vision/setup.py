from glob import glob

from setuptools import find_packages, setup

package_name = 'rm_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/weights', glob('weights/*.engine')),
        ('share/' + package_name + '/weights/Blue', glob('weights/Blue/*.engine')),
        ('share/' + package_name + '/weights/Red', glob('weights/Red/*.engine')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yao',
    maintainer_email='yao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detector_node = rm_vision.detector_node:main',
            'video_node = rm_vision.video_node:main',
        ],
    },
)
