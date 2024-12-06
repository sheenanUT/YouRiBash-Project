from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_assets'

robot_names = ['franka_panda', 'kinova', 'kuka_iiwa']
robot_files = [(os.path.join('share', package_name, 'robots/' + name),
                glob("robots/urdfs/robots/" + name + "/**/*.*", recursive=True)) for name in robot_names]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name, 'robots'), glob('robots/*/meshes/**.*', recursive=True))
    ] + robot_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sheneman',
    maintainer_email='sheneman23@utexas.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
