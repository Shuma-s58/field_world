import os
from glob import glob
from setuptools import setup

package_name = 'field_world'


def tree(src, dst):
    out = {}
    for root, _, files in os.walk(src):
        if files:
            out.setdefault(os.path.join(dst, os.path.relpath(root, src)),
                           []).extend(os.path.join(root, f) for f in files)
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
    ] + tree('models', 'share/' + package_name + '/models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Gazebo Classic field with a/b/c/d areas and randomized label boxes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'generate_world = field_world.generate_world:main',
        ],
    },
)
