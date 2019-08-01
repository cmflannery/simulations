"""A setuptools based setup module."""
# Always prefer setuptools over distutils
from setuptools import setup, find_packages
from os import path
# io.open is needed for projects that support Python 2.7
# It ensures open() defaults to text mode with universal newlines,
# and accepts an argument to specify the text encoding
# Python 3 only projects can skip this import
from io import open

with open('requirements.txt') as f:
    requirements = f.read().splitlines()

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, 'README.md'), encoding='utf-8') as f:
    long_description = f.read()

setup(
    name='simulations',
    # Versions should comply with PEP 440:
    # https://www.python.org/dev/peps/pep-0440/
    version='0.0.1',
    description='An opensource package for rocket simulations',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/cmflannery/simulations',
    author='cameron',
    author_email='cmflannery@ucla.edu',
    entry_points={
        'console_scripts': [
            'rocket-sim=simulations.interface:entry',
        ],
    },
)
