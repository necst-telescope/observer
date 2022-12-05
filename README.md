# observer

[![License](https://img.shields.io/badge/license-MIT-blue.svg?label=License&style=flat-square)](https://github.com/necst-telescope/observer/blob/main/LICENSE)

Graphical console of NECST system.

## Features

This library provides:

- Quick visualization of ROS 2 topics.
- More to come...

## Usage

> **Warning**
> Do not make the address public. This package currently runs on development server of
> Flask, which is not sufficiently secure, efficient, nor stable.

To use this, this following set-ups are required.

- Installation of Docker Engine
- `docker login ghcr.io` with your GitHub Personal Access Token

```shell
docker run -it --rm ghcr.io/necst-telescope/observer:latest
```

---

This library is using [Semantic Versioning](https://semver.org).
