from setuptools import find_packages, setup

package_name = "flux_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="nevlife",
    maintainer_email="nevlife000720@gmail.com",
    description="Republish flux channels as DDS topics for stock ROS 2 tools.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "bridge = flux_bridge.bridge:main",
        ],
        "ros2topic.verb": [
            "echo_flux = flux_bridge.verb.topic_echo_flux:TopicEchoFluxVerb",
            "hz_flux = flux_bridge.verb.topic_hz_flux:TopicHzFluxVerb",
        ],
        "ros2bag.verb": [
            "record_flux = flux_bridge.verb.bag_record_flux:BagRecordFluxVerb",
        ],
    },
)
