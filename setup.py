#!/usr/bin/env python
# -*- coding: utf-8 -*-
from setuptools import setup

setup(
    name="PythonSimulator",
    version="0.0.1",
    description="UnBall-IA Simulação de campo.",
    author="Hiago dos Santos",
    author_email="hiagop22@gmail.com",
    url="https://unball.github.io/",
    packages=["PythonSimulator"],
    install_requires=[
        "box2d>=2.3.10",
        "pygame>=2.0.0",
    ],
)