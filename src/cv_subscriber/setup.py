from setuptools import find_packages, setup

package_name = 'cv_subscriber'

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
    maintainer='sunrise',
    maintainer_email='jxudi@connect.ust.hk',
    description='Camera image subscriber using OpenCV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_subscriber = cv_subscriber.cv_suvscriber:main',
        ],
    },
)
