from enum import Enum


class CMD(Enum):
    PING = 0
    ECHO = 1
    GET_TIME_MILLIS = 2
    ENABLE_BUFFER = 3
    RETRIEVE_BUFFER = 4
    DISABLE_BUFFER = 5
