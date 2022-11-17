try:
    from importlib.metadata import version
except ImportError:
    from importlib_metadata import version

try:
    __version__ = version("ros2viz")
except:  # noqa: E722
    __version__ = "0.0.0"
