from setuptools import setup

package_name = 'odom_tf_broadcaster'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odom_tf_broadcaster.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Bridge odom topic to TF (odom->base_footprint/base_link) per robot namespace.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_tf_broadcaster = odom_tf_broadcaster.odom_tf_broadcaster:main',
        ],
    },
)
