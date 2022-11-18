ARG DISTRO=humble
FROM ros:${DISTRO}-ros-base
ARG DISTRO

ARG ROSPKG=""

SHELL ["/bin/bash", "-c"]
ENV $SHELL=/bin/bash

ENV POETRY_VIRTUALENVS_CREATE=false
ENV PATH=$PATH:/root/.local/bin

RUN apt-get update \
    && apt-get -y install curl git pciutils python3-pip ros-${DISTRO}-rmw-cyclonedds-cpp \
    && apt-get clean \
    && curl -sSL https://install.python-poetry.org | python3 -

ENV ROS2_WS=/root/ros2_ws
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY . /root/ros2viz

RUN ( cd /root/ros2viz && poetry install ) \
    && for __pkg in ${ROSPKG}; do git clone $__pkg $ROS2_WS/; done

RUN echo ". /opt/ros/${DISTRO}/setup.bash" >> /root/.bashrc \
    && curl -o /root/.bashgit https://raw.githubusercontent.com/oyvindstegard/bashgit/master/.bashgit \
    && echo "if [ -f /root/.bashgit ]; then . ~/.bashgit; fi" >> /root/.bashrc

ENTRYPOINT ["bash", "/ros_entrypoint.sh"]
CMD ["ros2viz"]
