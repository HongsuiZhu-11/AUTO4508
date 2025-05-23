from setuptools import setup, find_packages

package_name = 'vision_yolo_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # 自动发现 vision_yolo_detector
    include_package_data=True,  # 启用 package_data
    package_data={
        package_name: ['model/*.pt'],  # 包含模型文件
    },
    data_files=[
        ('share/' + package_name, ['package.xml', 'setup.cfg']),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'torch',
        'numpy',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Hongsui Zhu',
    maintainer_email='you@example.com',
    description='YOLOv8 detector node for cone/bucket identification',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_node = vision_yolo_detector.yolo_center_detector:main',
            'digit_node = vision_yolo_detector.digit_detector_node:main',
            'camera_saver_node = vision_yolo_detector.camera_saver_node:main',
            'unified_detector_node = vision_yolo_detector.unified_detector_node:main',
        ],
    }
)
