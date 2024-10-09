from setuptools import find_packages, setup

package_name = 'natnet_pub_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'natnet_pub_pkg.OptiTrackPubNode',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'optitrack_pub_node = natnet_pub_pkg.OptiTrackPubNode:main',
        ],
    },
    package_data={
        'natnet': ['*.py'],  # Include all Python files in the natnet directory
    },
)
