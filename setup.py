from pathlib import Path

from setuptools import find_packages, setup

package_name = 'object_rendering'

asset_files = [
    (
        'share/' + package_name + '/' + str(path.parent),
        [str(path)],
    )
    for path in Path('assets').rglob('*')
    if path.is_file()
]

launch_files = [
    (
        'share/' + package_name + '/launch',
        [str(path)],
    )
    for path in Path('launch').glob('*.py')
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + asset_files + launch_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yifan',
    maintainer_email='yifanluo2006@gmail.com',
    description=(
        'ROS 2 package for simulated object placement overlays'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_renderer = object_rendering.nodes.scene_renderer_node:main',
        ],
    },
)
