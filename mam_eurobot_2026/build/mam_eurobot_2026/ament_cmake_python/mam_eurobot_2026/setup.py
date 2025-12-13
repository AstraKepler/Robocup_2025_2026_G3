from setuptools import find_packages
from setuptools import setup

setup(
    name='mam_eurobot_2026',
    version='0.0.0',
    packages=find_packages(
        include=('mam_eurobot_2026', 'mam_eurobot_2026.*')),
)
