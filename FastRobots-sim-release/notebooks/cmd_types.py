from enum import Enum


class CMD(Enum):
    PING = 0
    ECHO = 1
    GET_TIME_MILLIS = 2
    ENABLE_ROBOT = 3
    DISABLE_ROBOT = 4
    ENABLE_BUFFER = 5
    RETRIEVE_BUFFER = 6
    DISABLE_BUFFER = 7
    SET_PID_GAINS = 8
    SET_MODE = 9
    SET_TARGET = 10
    SET_GYRO = 11
