from setuptools import setup, find_packages

setup(
    name='interview-client',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        'python-dotenv',
        'requests',
        'customtkinter',
        'pync; platform_system=="Darwin"',
        'plyer; platform_system=="Windows"',
    ],
    entry_points={
        'console_scripts': [
            'interview=interview_client.cli:main',
        ],
    },
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
    python_requires='>=3.7',
    zip_safe=True,
)
