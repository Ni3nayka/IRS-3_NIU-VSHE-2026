from setuptools import find_packages, setup

package_name = 'main'

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
    maintainer='ni3nayka',
    maintainer_email='53381511+Ni3nayka@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # extras_require={
    #     'test': [
    #         'pytest',
    #     ],
    # },
    entry_points={
        'console_scripts': [
            'main = main.main_node:main', # НОДА
            'minimal = main.main_minimal_node:main',
            'minimal_2 = main.main_minimal_2_node:main',
            'RC = main.main_RC_node:main',
            'RC_X = main.main_RC_X_node:main',
            'test = main.main_test_node:main',
        ],
    },
)
