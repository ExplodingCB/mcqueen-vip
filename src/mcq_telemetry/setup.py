from glob import glob

from setuptools import find_packages, setup

setup(
    name="mcq_telemetry",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/mcq_telemetry"]),
        ("share/mcq_telemetry", ["package.xml", "README.md"]),
        *[("share/mcq_telemetry/" + folder, glob(folder + "/*")) for folder in ("config", "launch", "layouts")],
    ],
    install_requires=["setuptools", "pyyaml"],
    tests_require=["pytest"],
    maintainer="Chase Culbertson",
    maintainer_email="culbertsonprime@gmail.com",
    description="Best-effort pit telemetry, Foxglove layouts and lap timing.",
    license="MIT",
    entry_points={"console_scripts": ["telemetry_node = mcq_telemetry.node:main"]},
)
