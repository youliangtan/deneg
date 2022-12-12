from setuptools import setup

package_name = 'deneg'

setup(
    name=package_name,
    version='2.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='youliang',
    author_email='tan_you_liang@hotmail.com',
    zip_safe=True,
    maintainer='youliang',
    maintainer_email='tan_you_liang@hotmail.com',
    description='Deneg Library',
    license='Apache License Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'test_agent = deneg.agent:test',
        ],
    },
)
