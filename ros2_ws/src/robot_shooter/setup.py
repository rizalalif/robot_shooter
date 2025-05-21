from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_shooter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/urdf', glob('urdf/*.urdf.xacro')),
        (f'share/{package_name}/worlds', glob('worlds/*.world')),
        (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
        (f'share/{package_name}/meshes', glob('meshes/*')),
        (f'share/{package_name}/config', glob('config/*')),
    ],
    install_requires=['setuptools','opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='rizalalif03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_node = robot_shooter.test_node:main',
            'sender = robot_shooter.test_publish:main',
            'listener = robot_shooter.test_subscribe:main',
            'image_sub = robot_shooter.image_sub:main',
            'image_pub = robot_shooter.image_pub:main',
            'camera_capture = robot_shooter.camera_capture:main',
            'target_deleter = robot_shooter.target_deleter:main',
        ],
    },
)
