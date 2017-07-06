cmake_minimum_required(VERSION 3.7)
project(CommunicationChannel)

set(CMAKE_C_STANDARD 99)

set(SOURCE_FILES
        CommunicationProtocol/protocol_keywords.h
        CRC_16-CCITT/crc_16ccitt.c
        CommunicationFunctions/serial_communication_functions.h
        CommunicationFunctions/serial_communication_functions.c
        main.c
        CRC_16-CCITT/crc_16ccitt.h)
add_executable(CommunicationChannel ${SOURCE_FILES})
