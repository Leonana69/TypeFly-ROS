from setuptools import find_packages, setup

package_name = 'typego'

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
    maintainer='guojun',
    maintainer_email='guojun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'basic_control = typego.basic_control:main',
            'video_stream = typego.video_stream:main',
        ],
    },
)
