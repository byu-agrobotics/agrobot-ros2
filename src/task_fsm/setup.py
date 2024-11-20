from setuptools import find_packages, setup

package_name = 'task_fsm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nelson Durrant',
    maintainer_email='snelsondurrant@gmail.com',
    description='Task FSMs for use by the BYU Agrobotics Team',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'collect_fsm = task_fsm.collect_fsm:main', TODO: Uncomment this line
            # 'handle_fsm = task_fsm.handle_fsm:main', TODO: Uncomment this line
            'navigate_fsm = task_fsm.navigate_fsm:main',
            'sort_fsm = task_fsm.sort_fsm:main',
        ],
    },
)
