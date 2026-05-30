FROM ghcr.io/necst-telescope/necst:v4.0.19

ENV PATH=$PATH:/root/.local/bin
RUN curl -sSL https://install.python-poetry.org | python3 - \
    && apt-get install python-is-python3

COPY . /root/observer

RUN cd /root/observer \
    && poetry install \
    && poetry run pip install -U astropy

ENV PATH=/root/observer/.venv/bin:$PATH

EXPOSE 8080

ENTRYPOINT ["bash", "/entrypoint.sh"]
CMD ["observer"]
