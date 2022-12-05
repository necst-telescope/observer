import ipaddress
import socket
from typing import List, Literal, Optional, overload

import psutil


@overload
def get_ip_address(all: Literal[True]) -> List[str]:
    ...


@overload
def get_ip_address(all: Literal[False]) -> Optional[str]:
    ...


def get_ip_address(all: bool = False):
    available_if_info = psutil.net_if_addrs()
    available_ipv4_addr_info = []
    _ = [
        available_ipv4_addr_info.extend(filter(lambda a: a.family == socket.AF_INET, v))
        for v in available_if_info.values()
    ]
    ipv4_addrs = (ipaddress.ip_address(x.address) for x in available_ipv4_addr_info)
    valid_addrs = filter(lambda x: x.is_private and (not x.is_loopback), ipv4_addrs)
    valid_addrs_str = map(str, valid_addrs)
    if all:
        return list(valid_addrs_str)
    try:
        return next(valid_addrs_str)
    except StopIteration:
        return
