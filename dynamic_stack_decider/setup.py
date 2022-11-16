from setuptools import find_packages
from setuptools import setup

package_name = 'dynamic_stack_decider'

setup(
    name=package_name,
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
    ],
    test_suite='tests',
    zip_safe=True,
    keywords=['ROS'],
    license='MIT',
    scripts=['scripts/dsd_parser.py'],
    entry_points={
        #  'console_scripts': [
            #  'dynamic_stack_decider = dynamic_stack_decider.parser:parse',
        #  ],
    }
)
