from importlib.metadata import version

__version__ = version("observer")

from . import address, client_manager, server  # noqa: F401
