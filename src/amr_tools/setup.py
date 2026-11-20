from setuptools import setup

package_name = 'amr_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Munavir Zaman',
    maintainer_email='you@example.com',
    description='AMR navigation helper nodes and benchmarking tools',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = amr_tools.cmd_vel_relay:main',
            'dynamic_obstacle_driver = amr_tools.dynamic_obstacle_driver:main',
            'nav_benchmark = amr_tools.nav_benchmark:main',
            'planner_benchmark = amr_tools.planner_benchmark:main',
            'analyze_results = amr_tools.analyze_results:main',
            'auto_mapper = amr_tools.auto_mapper:main',
            'compare_maps = amr_tools.compare_maps:main',
        ],
    },
)
