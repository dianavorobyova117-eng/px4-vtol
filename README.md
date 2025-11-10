# Introdcution
This is low-level controller implementation of the project `aerodynamic prior-free trajectory generation and tracking control for a tail-sitter UAV` 
(under review);

The aerodynamics plugin that used by the `gz_tailsitter` will be installed by the top-level ROS2 package, which is now private util the paper is published.

Note that the `docker` branch speficially remove the hareware compiler installtion dependencies, since this process in container building is very **framented** and **slow**.

# Installation
```shell
git clone --recursive https://github.com/WarriorHanamy/px4-vtol.git -b docker --depth=1
```

## build simulator container image
```shell
just build-px4
```



# Test Installation

## Run in container
```shell
just run-px4
```

In container bash 
```shell
runpx4 2
```
