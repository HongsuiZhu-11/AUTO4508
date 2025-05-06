from setuptools import setup
import glob

package_name = 'vision_yolo_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
        ('share/' + package_name + '/model', glob.glob('vision_yolo_detector/model/*')),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'torch',
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='YOLOv8 detector node for cone/bucket identification',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_node = vision_yolo_detector.yolo_center_detector:main',
        ],
    },
)
