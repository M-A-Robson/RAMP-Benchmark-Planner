#!/usr/bin/env python3

from distutils.core import setup

setup(name='planner',
      version='0.1',
      description='planning package',
      author='Mark Robson',
      author_email='mark.robson91@googlemail.com',
      packages=['sparc_planning',
                'sparc_planning/src',
                'beam_assembly'],
     )