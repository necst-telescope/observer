import ipaddress
import socket
from typing import List, Optional

import psutil


def list_ip_address() -> List[str]:
    available_if_info = psutil.net_if_addrs()
    valid_addresses = []
    for interface, if_info in available_if_info.items():
        for info in if_info:
            if info.family != socket.AF_INET:
                continue
            address = ipaddress.ip_address(info.address)
            if (not address.is_private) or address.is_loopback:
                continue
            valid_addresses.append((interface, str(address)))
    return valid_addresses


def get_ip_address(if_or_address: Optional[str] = None, /) -> Optional[str]:
    available = list_ip_address()
    if len(available) == 0:
        return
    if if_or_address is None:
        return available[0][1]

    interfaces = (x[0] for x in available)
    addresses = (x[1] for x in available)
    if if_or_address in addresses:
        return if_or_address
    if if_or_address in interfaces:
        address = filter(lambda x: x[0] == if_or_address, available)
        try:
            return next(address)[1]
        except StopIteration:
            return
