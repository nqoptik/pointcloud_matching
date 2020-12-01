# Build stage
FROM ubuntu:focal AS build

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
    && git clone https://github.com/libLAS/libLAS.git && cd libLAS && git checkout 1.8.1 \
    && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release .. && make && make install \
    && cd ../.. && rm -rf libLAS \
    && apt-get autoremove -y --purge \
    ca-certificates \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/pointcloud_matching

COPY include include
COPY src src
COPY CMakeLists.txt .

RUN mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make

# Production stage
FROM ubuntu:focal AS production

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
    && git clone https://github.com/libLAS/libLAS.git && cd libLAS && git checkout 1.8.1 \
    && mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Release .. && make && make install \
    && cd ../.. && rm -rf libLAS \
    && apt-get autoremove -y --purge \
    ca-certificates \
    cmake \
    g++ \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/pointcloud_matching

COPY --from=build /root/pointcloud_matching/build build

CMD [ "build/pre_process" ]
