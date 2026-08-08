from setuptools import setup

package_name = 'brock_operate'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imamura',
    maintainer_email='imamura@example.com',
    description='Brock operate node',
    license='Apache-2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'brock_operate = brock_operate.brock_operate:main',
        ],
    },
)