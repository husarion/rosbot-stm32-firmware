FROM python:3.9-slim AS stm32flash_builder

ARG user=husarion
# official releases are only for intel archs, so we need to build stm32flash from sources
RUN apt-get update && apt-get install -y \
        git \
        build-essential sudo

RUN useradd -ms /bin/bash ${user} && \
    echo "$user:$user" | chpasswd && \
    adduser ${user} sudo && \
    echo "$user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${user}
RUN sudo apt-get update && \
    sudo apt-get install --no-install-recommends -y \
    gdb clang clang-format cppcheck bash-completion python3-pip curl && \
    python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)" && \
    sudo ln -s /home/husarion/.platformio/penv/bin/platformio /bin/platformio