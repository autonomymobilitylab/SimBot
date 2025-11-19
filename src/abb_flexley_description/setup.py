from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'abb_flexley_description'

# Move this block ABOVE setup() call
model_files = []
for root, _, files in os.walk('models'):
    for file in files:
        model_files.append(os.path.join(root, file))

# Now call setup
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Recursively include all files from 'models'
        *[
            (os.path.join('share', package_name, os.path.dirname(f)), [f])
            for f in model_files
        ]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anis',
    maintainer_email='muhammad.1.anis@aalto.fi',
    description='ABB_FLEXLEY_TUG AMR Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sensor_publisher = abb_flexley_description.sensor_publisher:main',
        'control_node = abb_flexley_description.control_node:main',
        'lidar_frame_merger = abb_flexley_description.lidar_frame_merger:main',
        'keyboard_attach = abb_flexley_description.keyboard_attach:main',
        'attach_detach_key_control = abb_flexley_description.attach_detach_key_control:main',
        'multi_camera_stitcher = abb_flexley_description.multi_camera_stitcher:main',
    ],
    },
)

