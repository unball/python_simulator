#!/usr/bin/env python
# -*- coding: utf-8 -*-
from setuptools import setup

setup(
    name="PythonSimulator",
    version="0.1.3",
    description="UnBall-IA Simulação de campo.",
    author="Hiago dos Santos",
    author_email="hiagop22@gmail.com",
    url="https://unball.github.io/",
    packages=["PythonSimulator", "PythonSimulator.objects_on_field", "PythonSimulator.pygame_framework", "PythonSimulator.pygame_framework.backends"],
    install_requires=[
        "box2d>=2.3.2",
        "pygame>=2.0.0",
        "box2d-kengz==2.3.3"
    ],
)
