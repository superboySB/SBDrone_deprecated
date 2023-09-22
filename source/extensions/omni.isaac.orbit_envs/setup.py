# Copyright (c) 2022, NVIDIA CORPORATION & AFFILIATES, ETH Zurich, and University of Toronto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Installation script for the 'omni.isaac.orbit_envs' python package."""

import itertools
import os
import toml

from setuptools import setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # generic
    "numpy",
    "torch",
    "torchvision>=0.14.1",  # ensure compatibility with torch 1.13.1
    "protobuf==3.20.2",
    # gym
    "gym==0.21.0",
    "importlib-metadata==4.13.0",
    "setuptools==63.2.0",
    "wheel<0.40.0",
    # data collection
    "h5py",
]

# Extra dependencies for RL agents
EXTRAS_REQUIRE = {
    "rl_games": ["rl-games==1.5.2"],
    "rsl_rl": ["rsl_rl@git+https://github.com/leggedrobotics/rsl_rl.git"],
    "robomimic": ["robomimic@git+https://github.com/ARISE-Initiative/robomimic.git"],
}
# cumulation of all extra-requires
EXTRAS_REQUIRE["all"] = list(itertools.chain.from_iterable(EXTRAS_REQUIRE.values()))


# Installation operation
setup(
    name="omni-isaac-orbit_envs",
    author="NVIDIA, ETH Zurich, and University of Toronto",
    maintainer="Mayank Mittal",
    maintainer_email="mittalma@ethz.ch",
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    include_package_data=True,
    python_requires=">=3.7",
    install_requires=INSTALL_REQUIRES,
    extras_require=EXTRAS_REQUIRE,
    packages=["omni.isaac.orbit_envs"],
    classifiers=["Natural Language :: English", "Programming Language :: Python :: 3.7"],
    zip_safe=False,
)
