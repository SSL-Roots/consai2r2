from setuptools import setup

package_name = 'consai2r2_referee_wrapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='akshota',
    author_email='macakasit@gmail.com',
    maintainer='akshota',
    maintainer_email='macakasit@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Translate raw referee message to AI-friendly message',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'referee_wrapper = consai2r2_referee_wrapper.referee_wrapper:main',
        ],
    },
)
