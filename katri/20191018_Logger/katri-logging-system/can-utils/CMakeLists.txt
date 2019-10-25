cmake_minimum_required(VERSION 3.14)
project(katricandump C)

set(CMAKE_C_STANDARD 11)

add_executable(katricandump
        include/linux/can/bcm.h
        include/linux/can/error.h
        include/linux/can/gw.h
        include/linux/can/isotp.h
        include/linux/can/j1939.h
        include/linux/can/netlink.h
        include/linux/can/raw.h
        include/linux/can/vxcan.h
        include/linux/can.h
        candump.c
        lib.c)
