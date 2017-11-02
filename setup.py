#!/usr/bin/env python
from setuptools import setup

setup(name = "python-ec301lib",
      version = "0.0.1",
      description = ("Library for driving the SRS EC301 potentiostat/galvanostat."),
      author = "Alexander Bjoerling",
      author_email = "alexander.bjorling@maxiv.lu.se",
      license = "GPLv3",
      url = "http://www.maxiv.lu.se",
      packages =['ec301lib'],
      package_dir = {'':'src'},
     )
