import os

from ament_index_python.packages import get_package_prefix


def run(executable, argv):
    """Become flux2_bag's `executable`, so Ctrl-C and the exit code are the verb's own."""
    path = os.path.join(get_package_prefix("flux2_bag"), "lib", "flux2_bag", executable)
    os.execv(path, [path, *argv])
