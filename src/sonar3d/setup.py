from setuptools import find_packages, setup

package_name = 'sonar3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'protobuf==3.20.0',
        'requests==2.32.3'
        
    ],
    zip_safe=True,
    maintainer='Water Linked',
    maintainer_email='support@waterlinked.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'replay_file = sonar3d.file_replay:main',
            'sonar_publisher = sonar3d.multicast_listener:main'
        ],
    },
)
