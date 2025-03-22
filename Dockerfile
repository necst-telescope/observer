FROM ghcr.io/necst-telescope/necst:v4.0.18

ENV POETRY_VIRTUALENVS_CREATE=false
ENV PATH=$PATH:/root/.local/bin
RUN curl -sSL https://install.python-poetry.org | python3 - \
    && apt-get -y install python-is-python3 npm nodejs
RUN pip install -U astropy

RUN npm install n -g \
    && n 22.14.0 \
    && apt purge -y nodejs npm \
    && apt autoremove -y

COPY . /root/observer

RUN ( cd /root/observer && npm ci )

EXPOSE 8080

ENTRYPOINT ["bash", "/entrypoint.sh"]
CMD ["observer"]
