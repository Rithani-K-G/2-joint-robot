from setuptools import setup
from glob import glob
import os

package_name = 'two_joint_bringup'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Team',
    maintainer_email='maintainer@example.com',
    description='Gazebo bringup for a two-joint robot',
    license='Apache-2.0',
)
