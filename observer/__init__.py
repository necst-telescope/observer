from importlib.metadata import version

__version__ = version("observer")

from . import address, client, server  # noqa: F401
