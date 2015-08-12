#!/usr/bin/env python
# -*- coding: utf-8 -*-


try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup


with open('README.rst') as readme_file:
    readme = readme_file.read()

with open('HISTORY.rst') as history_file:
    history = history_file.read().replace('.. :changelog:', '')

requirements = [
    'numpy',
    'Pillow',
    'mlpy',
]

test_requirements = [
    'pytest',
]

import naobot

setup(
    name='naobot',
    version=naobot.__version__,
    description="Control of a Nao robot",
    long_description=readme + '\n\n' + history,
    author="Astrid Jackson",
    author_email='ajackons@eecs.ucf.edu',
    url='https://readthedocs.org/builds/naobot',
    download_url='https://github.com/evenmarbles/naobot',
    packages=[
        'naobot',
    ],
    package_dir={'naobot':
                 'naobot'},
    include_package_data=True,
    install_requires=requirements,
    license="MIT",
    zip_safe=False,
    keywords='NAO robot',
    classifiers=[
        'Development Status :: 2 - Pre-Alpha',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        "Programming Language :: Python :: 2",
        'Programming Language :: Python :: 2.7',
    ],
    test_suite='tests',
    tests_require=test_requirements
)
