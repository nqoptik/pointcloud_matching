FROM ubuntu:focal

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    cmake \
    g++ \
    git \
    libgeotiff-dev \
    libopencv-dev \
    libpcl-dev \
    make \
    && rm -rf /var/lib/apt/lists/*

RUN git clone https://github.com/libLAS/libLAS.git
RUN cd libLAS && git checkout 1.8.1
RUN mkdir -p libLAS/build && cd libLAS/build && cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DCMAKE_BUILD_TYPE=Release .. && make && make install
RUN rm -rf libLAS

WORKDIR /root/pointcloud_matching

COPY include include
COPY src src
COPY CMakeLists.txt .

RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make

RUN rm -rf include
RUN rm -rf src
RUN rm CMakeLists.txt
