from setuptools import find_packages, setup

package_name = 'capacitacion_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), 
    data_files=[
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('lib/' + package_name, ['src/Com_motores.py', 'src/Publisher_msg.py']),  # Add your executable scripts here
	],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diego',
    maintainer_email='diego@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Add any executable entries here if necessary
        ],
    },
)

