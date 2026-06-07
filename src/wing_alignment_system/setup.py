from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'wing_alignment_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YourName',
    maintainer_email='your@email.com',
    description='Wing alignment active search system',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goto_pose_driver = wing_alignment_system.goto_pose_driver:main',
            'mission_coordinator = wing_alignment_system.mission_coordinator:main',
            'multi_tracer_return_home = wing_alignment_system.multi_tracer_return_home:main',
            'cmd_scheduler = wing_alignment_system.cmd_scheduler_node:main',
            'cmd_watchdog = wing_alignment_system.cmd_watchdog_node:main',
            'multi_tracer_keyboard_control = wing_alignment_system.multi_tracer_keyboard_control:main',
            'slide_speed_sine_test_multi = wing_alignment_system.slide_speed_sine_test_multi:main',
            'slide_speed_sine_test = wing_alignment_system.slide_speed_sine_test:main',
            'common_rt = wing_alignment_system.common_rt:main',
            'tiaozi = wing_alignment_system.tiaozi:main',
            'mission_phase_client = wing_alignment_system.mission_phase_client:main',
            'real_machine_preflight = wing_alignment_system.real_machine_preflight:main',
        ],
    },
)
