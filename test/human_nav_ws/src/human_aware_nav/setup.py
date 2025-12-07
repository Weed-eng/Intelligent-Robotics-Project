from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'human_aware_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
	(os.path.join('share', package_name, 'resource'), glob('resource/*.urdf')),
	(os.path.join('share', package_name, 'worlds'), glob('worlds/*.wbt')),
	(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhoushiwang',
    maintainer_email='zhoushiwang@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'driver = human_aware_nav.driver:main', # 如果这行不在也加上，方便以后调试
    	'human_detector = human_aware_nav.human_detector:main', # <--- 添加这一行
		'kalman_tracker = human_aware_nav.kalman_tracker:main',
        'adaptive_safety = human_aware_nav.adaptive_safety:main',
        ],
    },
)
