from setuptools import setup

package_name = 'vision_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='ROS2 target detection service using depthai-sdk',
    license='MIT',
    entry_points={
        'console_scripts': [
            'detect_target_service = vision_interface.detect_target_service:main',
            'detect_target_client = vision_interface.detect_target_client:main',
        ],
    },
)
