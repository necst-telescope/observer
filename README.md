# observer

[![License](https://img.shields.io/badge/license-MIT-blue.svg?label=License&style=flat-square)](https://github.com/necst-telescope/observer/blob/main/LICENSE)
[![](https://ghcr-badge.deta.dev/necst-telescope/observer/latest_tag?label=Latest)](https://github.com/necst-telescope/observer/pkgs/container/observer)
[![](https://ghcr-badge.deta.dev/necst-telescope/observer/size?label=Size)](https://github.com/necst-telescope/observer/pkgs/container/observer)

Graphical console for NECST system.

## Features

This library provides:

- Quick visualization of ROS 2 topics.
- System-wide configuration / parameter file server.
- More to come...

## Usage

> **Warning**  
> Do not make the address public. This package currently runs on development server of
> Flask, which is not sufficiently secure, efficient, nor stable.

To run this server, the following set-ups are required.

- Installation of Docker Engine
- `docker login ghcr.io` with your GitHub Personal Access Token

Then run the following command on your terminal.

```shell
docker run --rm -v ~/.necst:/root/.necst --net=host ghcr.io/necst-telescope/observer:latest &
```

Optionally you can provide host's `<interface>` or `<ipaddr>` and unoccupied `<port>` to
use. The network interface must be the same as the one NECST communication uses.

```shell
docker run --rm -v ~/.necst:/root/.necst --net=host ghcr.io/necst-telescope/observer:latest observer -p <port> -i <interface or ipaddr> &
```

---

This library is using [Semantic Versioning](https://semver.org).
