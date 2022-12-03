FROM ghcr.io/necst-telescope/necst:v0.3.2

ENV POETRY_VIRTUALENVS_CREATE=false
ENV PATH=$PATH:/root/.local/bin
RUN curl -sSL https://install.python-poetry.org | python3 - \
    && apt-get install python-is-python3

COPY . /root/ros2viz

RUN ( cd /root/ros2viz && poetry install )

EXPOSE 8080

ENTRYPOINT ["bash", "/entrypoint.sh"]
CMD ["ros2viz"]
