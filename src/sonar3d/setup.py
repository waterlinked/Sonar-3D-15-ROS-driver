from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sonar3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'protobuf==3.20.0',
        'requests==2.32.3',
        'numpy'
    ],
    zip_safe=True,
    maintainer='Water Linked',
    maintainer_email='support@waterlinked.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar_publisher = sonar3d.multicast_listener:main'
        ],
    },
)
