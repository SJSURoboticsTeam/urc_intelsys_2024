from setuptools import find_packages, setup
from glob import glob

package_name = 'keyboard_typing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/resource', ['resource/full_keyboard_template_image.jpg', 'resource/template_image.jpg']),  # install image specifically
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='japjikbatra@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "autonomous_typing = keyboard_typing.autonomous_typing:main",
            "camera_node = keyboard_typing.autonomous_typing:main",
        ],
    },
)
