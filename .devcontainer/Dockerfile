FROM ghcr.io/necst-telescope/necst:v0.4.0

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

RUN apt-get update \
    && apt-get install -y \
        curl \
        git \
        gnupg2 \
        openssh-client \
        python-is-python3 \
        python3-pip \
        sudo \
        vim \
    && apt-get clean

ENV POETRY_VIRTUALENVS_CREATE=false
ENV PATH=$PATH:/root/.local/bin
RUN curl -sSL https://install.python-poetry.org | python3 - \
    && pip3 install \
        "black>=22.3" \
        "flake8>=3.8" \
        "ipykernel>=6.3" \
        "myst-parser==0.18.0" \
        "notebook>=6.4" \
        "pydata-sphinx-theme==0.7.2" \
        "pytest>=7.2" \
        "pytest-cov==3.0.0" \
        "sphinx==4.5.0"

RUN curl -o /root/.bashgit https://raw.githubusercontent.com/oyvindstegard/bashgit/master/.bashgit \
    && echo "if [ -f /root/.bashgit ]; then . ~/.bashgit; fi" >> /root/.bashrc
