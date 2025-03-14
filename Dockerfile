FROM ghcr.io/necst-telescope/necst:v4.0.2

ENV POETRY_VIRTUALENVS_CREATE=false
ENV PATH=$PATH:/root/.local/bin
RUN curl -sSL https://install.python-poetry.org | python3 - \
    && apt-get install python-is-python3
RUN pip install -U astropy

COPY . /root/observer

RUN ( cd /root/observer && poetry install )

EXPOSE 8080

ENTRYPOINT ["bash", "/entrypoint.sh"]
CMD ["observer"]
