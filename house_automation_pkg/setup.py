from setuptools import find_packages, setup

package_name = 'house_automation_pkg'

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
    maintainer='enzo',
    maintainer_email='enzo.perrier@etu.unice.fr',
    description='Package proposant un exemple s utilisation de ros2 pour de la domotique',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'app = house_automation_pkg.app:main',
        	'bp1 = house_automation_pkg.bp1:main',
        	'LED = house_automation_pkg.servo:main',
        	'servo = house_automation_pkg.servo:main',
        	'temperature = house_automation_pkg.temperature:main',
        ],
    },
)
