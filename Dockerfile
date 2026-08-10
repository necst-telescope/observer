FROM ghcr.io/necst-telescope/necst:v4.0.20

ENV PATH=$PATH:/root/.local/bin
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV GIT_TERMINAL_PROMPT=0

RUN curl -sSL https://install.python-poetry.org | python3 - \
    && apt-get update \
    && apt-get install -y python-is-python3

RUN git clone https://github.com/necst-telescope/observer.git /root/observer

RUN cd /root/observer \
    && poetry config virtualenvs.in-project true \
    && poetry config virtualenvs.options.system-site-packages true \
    && poetry install \
    && poetry run pip install -U astropy

ENV PATH=/root/observer/.venv/bin:$PATH

EXPOSE 8080

ENTRYPOINT ["bash", "/entrypoint.sh"]
CMD ["bash"]