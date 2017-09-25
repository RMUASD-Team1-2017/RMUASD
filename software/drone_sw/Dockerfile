FROM ubuntu:xenial

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /usr/src/
ADD . /usr/src/app/
WORKDIR /usr/src/app/

RUN apt update -y && apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf cmake -y
RUN mkdir -p build
WORKDIR /usr/src/app/build
RUN cmake .. -DCMAKE_TOOLCHAIN_FILE=../CMakeToolchainFile.txt
RUN make -j4
