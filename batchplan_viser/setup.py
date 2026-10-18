from setuptools import setup

setup(
    name="batchplan_viser",
    version="0.1.0",
    packages=["batchplan_viser"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/batchplan_viser"]),
        ("share/batchplan_viser", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    license="MIT OR Apache-2.0",
    entry_points={"console_scripts": ["batchplan_viser = batchplan_viser.app:main"]},
)
