"""The options ros2 bag record and play build, handed to the C++ programs as YAML.

ros2bag parses and checks the arguments and fills rosbag2_py's StorageOptions, RecordOptions and
PlayOptions in its verbs' main(). That main() runs here with its Recorder or Player replaced, so
the options are the ones ros2 bag would have run with. The YAML keys are the ones rosbag2's YAML
converters read (rosbag2_storage storage_options.cpp, rosbag2_transport record_options.cpp and
play_options.cpp); the C++ programs decode it with those converters.
"""

import datetime
import math

import yaml


class _Built(Exception):
    def __init__(self, options):
        super().__init__()
        self.options = options


def built_by(module, name, verb, args):
    """The options `verb`'s main() hands to `module`.`name`, or the error it returned first."""
    real = getattr(module, name)

    def stand_in(*options):
        raise _Built(options)

    setattr(module, name, stand_in)
    try:
        error = verb.main(args=args)
    except _Built as built:
        return built.options
    finally:
        setattr(module, name, real)
    return error or "ros2 bag stopped before it built its options"


def duration(ns):
    """rclcpp::Duration as rosbag2's YAML holds it."""
    return {"sec": ns // 1_000_000_000, "nsec": ns % 1_000_000_000}


def seconds(s):
    return duration(round(s * 1e9))


def qos(profile):
    """An rclpy QoSProfile as rosbag2_storage's YAML holds it, every policy spelled out."""
    return {
        "history": profile.history.short_key,
        "depth": profile.depth,
        "reliability": profile.reliability.short_key,
        "durability": profile.durability.short_key,
        "deadline": duration(profile.deadline.nanoseconds),
        "lifespan": duration(profile.lifespan.nanoseconds),
        "liveliness": profile.liveliness.short_key,
        "liveliness_lease_duration": duration(profile.liveliness_lease_duration.nanoseconds),
        "avoid_ros_namespace_conventions": profile.avoid_ros_namespace_conventions,
    }


def qos_overrides(overrides):
    return {topic: qos(profile) for topic, profile in overrides.items()}


STORAGE_KEYS = (
    "uri",
    "storage_id",
    "max_bagfile_size",
    "max_bagfile_duration",
    "max_cache_size",
    "storage_preset_profile",
    "storage_config_uri",
    "snapshot_mode",
    "start_time_ns",
    "end_time_ns",
    "custom_data",
)


def storage(s):
    return {k: getattr(s, k) for k in STORAGE_KEYS}


# The rosbag2_py field of each YAML key whose name differs, and how its value is written.
RECORD_KEYS = {
    "all_topics": "all_topics",
    "all_services": "all_services",
    "is_discovery_disabled": "is_discovery_disabled",
    "topics": "topics",
    "topic_types": "topic_types",
    "services": "services",
    "exclude_topics": "exclude_topics",
    "exclude_topic_types": "exclude_topic_types",
    "exclude_services": "exclude_service_events",
    "rmw_serialization_format": "rmw_serialization_format",
    "regex": "regex",
    "exclude_regex": "exclude_regex",
    "node_prefix": "node_prefix",
    "compression_mode": "compression_mode",
    "compression_format": "compression_format",
    "compression_queue_size": "compression_queue_size",
    "compression_threads": "compression_threads",
    "compression_threads_priority": "compression_threads_priority",
    "include_hidden_topics": "include_hidden_topics",
    "include_unpublished_topics": "include_unpublished_topics",
    "ignore_leaf_topics": "ignore_leaf_topics",
    "start_paused": "start_paused",
    "use_sim_time": "use_sim_time",
    "disable_keyboard_controls": "disable_keyboard_controls",
}


def record(r):
    out = {key: getattr(r, field) for key, field in RECORD_KEYS.items()}
    out["topic_polling_interval"] = {
        "milliseconds": r.topic_polling_interval // datetime.timedelta(milliseconds=1)
    }
    out["topic_qos_profile_overrides"] = qos_overrides(r.topic_qos_profile_overrides)
    return out


PLAY_KEYS = {
    "read_ahead_queue_size": "read_ahead_queue_size",
    "node_prefix": "node_prefix",
    "rate": "rate",
    "topics_to_filter": "topics_to_filter",
    "services_to_filter": "services_to_filter",
    "regex_to_filter": "regex_to_filter",
    "exclude_regex_to_filter": "exclude_regex_to_filter",
    "exclude_topics": "exclude_topics_to_filter",
    "exclude_services": "exclude_service_events_to_filter",
    "clock_publish_frequency": "clock_publish_frequency",
    "clock_publish_on_topic_publish": "clock_publish_on_topic_publish",
    "clock_trigger_topics": "clock_topics",
    "start_paused": "start_paused",
    "disable_keyboard_controls": "disable_keyboard_controls",
    "disable_loan_message": "disable_loan_message",
    # Not read by Jazzy's PlayOptions decoder: play.cpp reads these itself.
    "loop": "loop",
    "topic_remapping_options": "topic_remapping_options",
    "publish_service_requests": "publish_service_requests",
}


def play(p):
    out = {key: getattr(p, field) for key, field in PLAY_KEYS.items()}
    out["topic_qos_profile_overrides"] = qos_overrides(p.topic_qos_profile_overrides)
    out["delay"] = seconds(p.delay)
    out["playback_duration"] = seconds(p.playback_duration)
    out["start_offset"] = seconds(p.start_offset)
    # Integers the decoder reads as a Duration and stores as its nanoseconds.
    out["playback_until_timestamp"] = duration(p.playback_until_timestamp)
    out["wait_acked_timeout"] = duration(p.wait_acked_timeout)
    out["service_requests_source"] = int(p.service_requests_source)
    return out


def dump(options):
    return yaml.safe_dump(options, default_flow_style=True, width=math.inf)
