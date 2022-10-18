FROM python:3.9-slim AS stm32flash_builder


ARG USERNAME=husarion
ARG USER_UID=1001
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN sudo apt-get update && \
    sudo apt-get install --no-install-recommends -y \
    build-essential sudo git cmake gdb clang clang-format cppcheck bash-completion python3-pip curl gcc-arm-none-eabi && \
    python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)" && \
    ln -s /home/husarion/.platformio/penv/bin/platformio /bin/pio && \
    pip install -U colcon-common-extensions

# Build core2-flasher
RUN git clone https://github.com/husarion/flasher && \
    cd flasher && git submodule update --init --recursive && \
    mkdir build && cd build && cmake .. && make && \
    ln -s /flasher/build/core2-flasher /bin/core2-flasher

# Build stm32 flasher
RUN git clone https://github.com/husarion/stm32loader && \
    cd stm32loader && sudo python setup.py install 