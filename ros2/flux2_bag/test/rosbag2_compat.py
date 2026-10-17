"""The rosbag2 the tests run against: 0.15 on Humble, 0.26 on Jazzy."""

import re
from pathlib import Path

import rosbag2_py
from ament_index_python.packages import get_package_share_directory

_XML = Path(get_package_share_directory("rosbag2_py")) / "package.xml"
MINOR = int(re.search(r"<version>0\.(\d+)\.", _XML.read_text())[1])


def topic_metadata(n, name, type_):
    """Topics are numbered from 0.26 on."""
    if MINOR >= 26:
        return rosbag2_py.TopicMetadata(id=n, name=name, type=type_, serialization_format="cdr")
    return rosbag2_py.TopicMetadata(name=name, type=type_, serialization_format="cdr")
