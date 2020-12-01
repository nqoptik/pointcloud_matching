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

RUN git clone https://github.com/libLAS/libLAS.git && cd libLAS && git checkout 1.8.1 \
    && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make && make install \
    && cd ../.. && rm -rf libLAS

WORKDIR /root/pointcloud_matching

COPY include include
COPY src src
COPY CMakeLists.txt .

RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make

RUN rm -rf include
RUN rm -rf src
RUN rm CMakeLists.txt
