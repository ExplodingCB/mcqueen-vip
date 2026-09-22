from setuptools import find_packages, setup

package_name = "mcq_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/sim_default.yaml", "config/kart_provisional.yaml"]),
    ],
    package_data={"mcq_sim": ["web/*.html", "web/*.css", "web/*.js"]},
    install_requires=["numpy>=1.26", "scipy>=1.11", "pyyaml>=6", "pillow>=10"],
    zip_safe=True,
    maintainer="ExplodingCB",
    maintainer_email="explodingcommandblock@gmail.com",
    description="Kart simulator, planner prototype and closed-loop harness for McQueen VIP.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mcq_sim = mcq_sim.__main__:main",
            "sim_node = mcq_sim.sim_node:main",
            "planner_node = mcq_sim.planner_node:main",
        ]
    },
)
