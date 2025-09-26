cmake_minimum_required(VERSION 3.0.2)
project(hightorque_robot)

# Find catkin
add_subdirectory(lib/serial_cmake)
add_subdirectory(lib/lcm)
add_subdirectory(lib/pybind11-2.5.0)

# find_package(serial REQUIRED)
# find_package(lcm REQUIRED)
find_package(yaml-cpp REQUIRED)

if(DEFINED ENV{CONDA_PREFIX})
    set(Python_ROOT_DIR $ENV{CONDA_PREFIX})
    message(STATUS "Using Conda environment: ${Python_ROOT_DIR}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.venv")
    # 检查当前目录下的.venv虚拟环境
    set(Python_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.venv")
    message(STATUS "Using Python virtual environment from .venv directory: ${Python_ROOT_DIR}")
else()
    message(WARNING "No Conda or .venv environment found, using system Python")
endif()

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

if(Python3_FOUND)
    message(STATUS "Found Python3: ${Python3_EXECUTABLE} (version: ${Python3_VERSION})")
else()
    message(FATAL_ERROR "Python3 not found. Please ensure a virtual environment or system Python is accessible.")
endif()

# Find pybind11 for Python bindings
set(Python_ROOT_DIR $ENV{CONDA_PREFIX})
# Try multiple methods to find pybind11
# find_package(pybind11 REQUIRED)

if(NOT pybind11_FOUND)
    # Try to find pybind11 via Python
    find_package(Python3 COMPONENTS Interpreter Development QUIET)
    if(Python3_FOUND)
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
            OUTPUT_VARIABLE pybind11_DIR
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
        if(pybind11_DIR)
            find_package(pybind11 QUIET HINTS ${pybind11_DIR})
        endif()
    endif()
endif()

if(pybind11_FOUND)
    message(STATUS "Found pybind11: ${pybind11_VERSION}")
    set(BUILD_PYTHON_BINDINGS ON)
else()
    message(STATUS "pybind11 not found. Install with: pip install pybind11")
    message(STATUS "Python bindings will not be built")
    set(BUILD_PYTHON_BINDINGS OFF)
endif()

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(serial_SRCS
    src/serial_driver.cpp
    src/parse_robot_params.cpp
    src/hardware/motor.cpp
    src/hardware/canport.cpp
    src/hardware/canboard.cpp
    src/hardware/robot.cpp
    src/crc/crc8.cpp
    src/crc/crc16.cpp
)

find_library(SERIALPORT_LIBRARY serialport)

if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

include_directories(
  include
  include/crc
  include/hardware
  msg/motor_msg
  /home/tang/miniconda3/envs/pinocchio_env/lib/python3.10/site-packages/pybind11/include
)

add_library(hightorque_robot SHARED ${serial_SRCS})
target_link_libraries(hightorque_robot rt pthread ${SERIALPORT_LIBRARY} serial lcm yaml-cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/msg)

add_executable(motor_run example/motor_run.cpp)
target_link_libraries(motor_run hightorque_robot)

add_executable(motor_msg_subscriber example/motor_msg_subscriber.cpp)
target_link_libraries(motor_msg_subscriber hightorque_robot)

# Python bindings
if(BUILD_PYTHON_BINDINGS)
    pybind11_add_module(hightorque_robot_py python/pybind_wrapper.cpp)
    target_link_libraries(hightorque_robot_py PRIVATE hightorque_robot)
    target_include_directories(hightorque_robot_py PRIVATE 
        ${CMAKE_CURRENT_SOURCE_DIR}/include 
        ${CMAKE_CURRENT_SOURCE_DIR}/include/hardware
        ${CMAKE_CURRENT_SOURCE_DIR}/include/crc
        ${CMAKE_CURRENT_SOURCE_DIR}/msg/motor_msg
    )
    
    # Set properties for Python module
    target_compile_definitions(hightorque_robot_py PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
    
    # Install Python module
    install(TARGETS hightorque_robot_py
        COMPONENT python
        LIBRARY DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/python
    )

    add_custom_target(custom_install
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:hightorque_robot_py> ${CMAKE_CURRENT_SOURCE_DIR}/../python
        DEPENDS hightorque_robot_py
    )
endif()

