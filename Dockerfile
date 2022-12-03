FROM ghcr.io/necst-telescope/necst:v0.3.2

ENV POETRY_VIRTUALENVS_CREATE=false
ENV PATH=$PATH:/root/.local/bin

COPY . /root/ros2viz

RUN ( cd /root/ros2viz && poetry install )

ENTRYPOINT ["bash", "/entrypoint.sh"]
CMD ["ros2viz"]
