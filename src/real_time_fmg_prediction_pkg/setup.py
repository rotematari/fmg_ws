from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'real_time_fmg_prediction_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'natnet'), glob('natnet/*')),
        (os.path.join('share', package_name,'model'), glob('models/model/*')),
        (os.path.join('share', package_name,'model'), glob('models/model/trained_models/transformer_model/*')),
        (os.path.join('share', package_name,'saved_models'), glob('saved_models/transformer_model/*')),
        

        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotics20',
    maintainer_email='rotem.atri@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RealTimeFmgPredictionNode = real_time_fmg_prediction_pkg.RealTimeFmgPredictionNode:main'
        ],
    },
)
