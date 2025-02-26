from setuptools import find_packages, setup

package_name = 'exploration_sim_utils'

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
    maintainer='Marshall Vielmetti',
    maintainer_email='mvielmet@umich.edu',
    description='Python utility package for exploration simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_grapher = exploration_sim_utils.PathGrapher:main',
            'fit_splines = exploration_sim_utils.FitSplines:main',
        ],
    },
)
