FROM ghcr.io/necst-telescope/necst:v4.0.18

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

RUN apt-get update \
    && apt-get install -y \
        curl \
        git \
        gnupg2 \
        openssh-client \
        sudo \
        vim \
        npm \
        nodejs \
    && apt-get clean

RUN npm install n -g \
    && n 22.14.0 \
    && apt purge -y nodejs npm \
    && apt autoremove -y

ENV PATH=$PATH:/root/.local/bin

RUN curl -o /root/.bashgit https://raw.githubusercontent.com/oyvindstegard/bashgit/master/.bashgit \
    && echo "if [ -f /root/.bashgit ]; then . ~/.bashgit; fi" >> /root/.bashrc