from setuptools import find_packages, setup

package_name = 'my_xarm6_app'

setup(
    name=package_name,
    version='0.0.0',
    #packages=find_packages(exclude=['test']),
    packages=[package_name, f'{package_name}.motion'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'scipy', 'open3d', 'pyyaml', 'transforms3d'],
    zip_safe=True,
    maintainer='fra',
    maintainer_email='ros.master.ai@gmail.com',
    description='Application package for xArm6: pick & place and vision with MoveIt',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_ik = my_xarm6_app.motion.test_ik:main',
            'move_to_pose = my_xarm6_app.motion.move_to_pose:main',
            'pick_place = my_xarm6_app.motion.pick_place:main'
        ],
    },
)
