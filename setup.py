from setuptools import find_packages, setup

package_name = 'adaptive_coverage'

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
    maintainer='adwaith',
    maintainer_email='adwaith@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coverage_controller = adaptive_coverage.nodes.coverage_controller:main',
            'obstacle_avoidance = adaptive_coverage.nodes.obstacle_avoidance:main'

        ],
    },
)
