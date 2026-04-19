from setuptools import setup, find_packages

setup(
    name="vma",
    version="0.1.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    entry_points={
        "console_scripts": [
            "vma=vma.cli.main:main",
        ],
    },
    install_requires=[
        "numpy>=1.24",
        "scipy>=1.10",
        "matplotlib>=3.7",
        "streamlit>=1.30",
    ],
)
