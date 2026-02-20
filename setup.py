#!/usr/bin/env python3
"""Setup configuration for sdk2_navigation module."""

from setuptools import setup, find_packages

setup(
    name="sdk2_navigation",
    version="1.0.0",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.20.0",
    ],
    python_requires=">=3.8",
)
