

FROM espressif/idf:v5.1.2

ENV DEBIAN_FRONTEND noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt update -q && \
    apt install -yq sudo lsb-release gosu nano locales locales-all && \
    rm -rf /var/lib/apt/lists/*


ARG TZ_ARG="America/Bahia"
ENV TZ=$TZ_ARG
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone


COPY ./install_micro_ros_deps_script.sh /install_micro_ros_deps_script.sh

RUN mkdir -p /tmp/install_micro_ros_deps_script && mv /install_micro_ros_deps_script.sh /tmp/install_micro_ros_deps_script/ && \
    IDF_EXPORT_QUIET=1 /tmp/install_micro_ros_deps_script/install_micro_ros_deps_script.sh && \
    rm -rf /var/lib/apt/lists/*

RUN /usr/bin/pip3 --no-cache-dir install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
ARG USER_ID=espidf

RUN useradd --create-home --home-dir /home/$USER_ID --shell /bin/bash --user-group --groups adm,sudo,dialout $USER_ID && \
    echo $USER_ID:$USER_ID | chpasswd && \
    echo "$USER_ID ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN echo "source $IDF_PATH/export.sh" >> /etc/bash.bashrc

ARG USER_ID
USER $USER_ID
