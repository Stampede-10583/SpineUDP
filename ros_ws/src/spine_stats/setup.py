from setuptools import setup

package_name = 'spine_stats'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FRC Team 10583',
    maintainer_email='jameskorten4@gmail.com',
    description='Spine link NT4 stats publisher and rotating JSONL logger',
    license='BSD-3-Clause',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'spine_stats = spine_stats.node:main',
        ],
    },
)
