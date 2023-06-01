"""Installation script for the 'kyonrlstepping' python package."""

from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from setuptools import setup, find_packages

import os

root_dir = os.path.dirname(os.path.realpath(__file__))

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # RL
#    "gym==0.23.1",
#    "torch",
    "termcolor",
    "hydra-core>=1.1",
    ]

# Installation operation
setup(
    name="kyonrlstepping",
    author="AndPatr",
    version="1.0.0",
    description="",
    keywords=["kyon", "stepping", "rl"],
    include_package_data=True,
    python_requires=">=3.7",
    install_requires=INSTALL_REQUIRES,
    packages=find_packages("."),
    classifiers=["Natural Language :: English", "Programming Language :: Python :: 3.6, 3.7, 3.8"],
    zip_safe=False,
)

# EOF