from setuptools import setup

package_name = 'autorccar_gcs'
submodules = "autorccar_gcs/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gspark',
    maintainer_email='pks87@nate.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autorccar_gcs = autorccar_gcs.pyqt_gcs_with_pyqtgraph:main',
        ],
    },
)
