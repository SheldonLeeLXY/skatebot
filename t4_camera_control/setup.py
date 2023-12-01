from setuptools import setup

package_name = 't4_camera_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pipebot',
    maintainer_email='a.j.blight@leeds.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_node = t4_camera_control.image_subscriber:main",
            "teleop_node = t4_camera_control.keyboard_teleop:main",
        ],
    },
)
