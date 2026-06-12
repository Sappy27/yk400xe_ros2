from setuptools import find_packages, setup

package_name = 'yk400xe_examples'

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
    maintainer='vincent',
    maintainer_email='vboufaroua@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'draw_yamaha_ros= yk400xe_examples.draw_yamaha_ros:main',
            'pick_n_place= yk400xe_examples.pick_n_place:main'
        ],
    },
)
