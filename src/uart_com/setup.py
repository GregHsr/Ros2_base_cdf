from setuptools import setup, SetuptoolsDeprecationWarning
import warnings

warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)

package_name = 'uart_com'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '/resources'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clément Ramirez',
    maintainer_email='clement@clementramirez.fr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uart_com_node = uart_com.uart_com_node:main'
        ],
    },
)
