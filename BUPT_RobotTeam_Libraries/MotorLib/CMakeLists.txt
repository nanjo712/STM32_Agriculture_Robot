cmake_minimum_required(VERSION 3.24)



set(
        MotorLib_SRC
        #BASE
        ${CMAKE_CURRENT_SOURCE_DIR}/dji_boardv2_can.c
        ${CMAKE_CURRENT_SOURCE_DIR}/dji_boardv2_can.h
)

set(MotorLib ${MotorLib_SRC} PARENT_SCOPE)
set(
        MotorLib_INC PARENT_SCOPE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
