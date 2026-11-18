from setuptools import find_packages, setup

package_name = 'my_tiago_tasks'

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
    maintainer='promise',
    maintainer_email='promzy007adiole@yahoo.com',
    description='Task scripts for my_tiago (navigation, manipulation)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'go_to_pose = my_tiago_tasks.go_to_pose:main',
            'robot_demo = my_tiago_tasks.robot_demo:main',
            'exercise1 = my_tiago_tasks.exercise1:main',
            'perception_demo = my_tiago_tasks.perception_demo:main',
            'pick_demo = my_tiago_tasks.pick_demo:main',
            'exercise2 = my_tiago_tasks.exercise2:main',
        ],
    },
)
