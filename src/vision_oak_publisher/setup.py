from setuptools import setup

package_name = 'vision_oak_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ROS 2 publisher for OAK camera using DepthAI',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oak_camera_publisher = vision_oak_publisher.oak_camera_node:main',
        ],
    },
)
