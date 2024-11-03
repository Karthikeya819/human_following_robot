from setuptools import find_packages, setup

package_name = 'human_following_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (('share/' + package_name + '/models', ['models/yolov8n.pt']))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karthikeya',
    maintainer_email='davasamkarthikeya@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "follow_human = human_following_robot.follow_human_member_function:main"
        ],
    },
)
