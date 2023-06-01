# Lidar-Ground-Removal
This repository provides a solution for "ground removal" from point cloud data.

It utilizes the Carla Server and C++ Carla client library to acquire point cloud data.

## Installation
### 0. download carla-server docker image
```bash
docker pull carlasim/carla:0.9.13
```

### 1. build LibCarla
[https://github.com/carla-simulator/carla/releases/tag/0.9.13](https://github.com/carla-simulator/carla/releases/tag/0.9.13)

- download source code(zip)
```bash
unzip carla-0.9.13
cd carla-0.9.13
```
- modify 'Util/BuildTools/Setup.sh' script.
  - replace XERCESC_VERSION=3.2.4
  - replace CARLA_VERSION="0.9.13"

```bash
sudo apt update && sudo apt install clang-8
make setup
make LibCarla
```

### 2. git clone
```
git clone https://github.com/LimHaeryong/Lidar-Ground-Removal.git
cd Lidar-Ground-Removal
cp -r /path/to/carla-0.9.13/PythonAPI/carla/dependencies .
mv dependencies libcarla-install
```

### Tested Environment
- Ubuntu 20.04
- Nvidia RTX 3070 / driver version : 510
### Dependencies
- docker, nvidia-docker
- Cmake
- PCL

## How to run
 
- terminal 1
```bash
docker run --rm --privileged --gpus all --runtime=nvidia  --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh
```
- terminal 2
```bash
mkdir build && cd build
cmake .. && make -j(nproc)
./Lidar-Ground-Removal
```