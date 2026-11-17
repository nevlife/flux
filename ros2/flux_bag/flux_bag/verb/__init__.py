import os

from ament_index_python.packages import get_package_prefix


def run(executable, argv):
    """Become flux_bag's `executable`, so Ctrl-C and the exit code are the verb's own."""
    path = os.path.join(get_package_prefix("flux_bag"), "lib", "flux_bag", executable)
    os.execv(path, [path, *argv])


def run_with_options(executable, options, ros_args):
    """run() with the YAML `options` as a file: rcl takes any argument with := for a remap."""
    fd = os.memfd_create("flux_bag_options", 0)  # kept open across exec
    with open(fd, "w", closefd=False) as f:
        f.write(options)
    run(executable, [f"/dev/fd/{fd}", *ros_args])
