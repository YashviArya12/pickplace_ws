from setuptools import find_packages, setup

package_name = 'pickplace_manipulation'

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
    maintainer='yashvi',
    maintainer_email='yashviarya012@gmail.com',
    description='Manipulation nodes for the pick-and-place project',
    license='MIT',
    tests_require=['pytest'],
        entry_points={
        'console_scripts': [
            'move_above_cube = pickplace_manipulation.move_above_cube:main',
            'pick_place_sequence = pickplace_manipulation.pick_place_sequence:main',
        ],
    },
)
