from setuptools import find_packages, setup

package_name = 'rl_package'

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
    maintainer='nair-group',
    maintainer_email='15696061+abiantorres@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'myosuite_publisher = rl_package.myosuite_publisher:main',
            'exo_publisher = rl_package.exo_publisher:main',
            'aggregator = rl_package.aggregator:main',
            'rl_node = rl_package.rl_node:main',
            'rl_node_v2 = rl_package.rl_node_v2:main',
            'rl_node_v3 = rl_package.rl_node_v3:main',
            'rl_node_v4 = rl_package.rl_node_v4:main',
            'rl_node_v5 = rl_package.rl_node_v5:main',
            'rl_node_v6 = rl_package.rl_node_v6:main',
            'rl_node_v7 = rl_package.rl_node_v7:main',
            'rl_training_node = rl_package.torchRL_gym_nair_PPO:main',
            'talker = rl_package.talker:main',
            'listener = rl_package.listener:main',
            'training_ros2socket = rl_package.training_ros2socket:main',
        ],
    },
)
