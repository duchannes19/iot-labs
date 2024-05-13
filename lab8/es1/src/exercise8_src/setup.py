from setuptools import find_packages, setup

package_name = 'exercise8_src'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fede3751',
    maintainer_email='ftrombetti@hotmail.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_controller = exercise8_src.drone_controller:main',
            'drone_client = exercise8_src.drone_client:main',
        ],
    },
)
