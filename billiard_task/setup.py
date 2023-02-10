from setuptools import setup

package_name = 'billiard_task'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andy',
    maintainer_email='N2107687J@e.ntu.edu.sg',
    description='TODO: Package description',
    license='BSD-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sample_node = billiard_task.sample_node:main'
        ],
    },
)
