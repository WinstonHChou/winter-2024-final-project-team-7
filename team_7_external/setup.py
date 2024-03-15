from setuptools import setup
import os
from glob import glob

package_name = 'team_7_external'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # (os.path.join('lib/python3.8/site-packages', package_name, 'razorIMU_9dof'), glob(package_name + '/razorIMU_9dof/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='w3chou@ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_corrected_node = team_7_external.scan_correction:main',
            'Seeed_imu_node = team_7_external.Seeed_imu:main',
        ],
    },
)
