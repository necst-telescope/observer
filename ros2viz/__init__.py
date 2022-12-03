from importlib.metadata import version

__version__ = version("ros2viz")

from . import address, client, server  # noqa: F401
