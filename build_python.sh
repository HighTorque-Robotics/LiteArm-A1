#!/bin/bash

# Build script for hightorque_robot with Python bindings

echo "Building hightorque_robot with Python bindings..."

# Check if pybind11 is installed
python3 -c "import pybind11" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "pybind11 not found. Installing..."
    pip3 install pybind11
    if [ $? -ne 0 ]; then
        echo "Failed to install pybind11. You might need to run:"
        echo "  sudo pip3 install pybind11"
        echo "  or"
        echo "  pip3 install --user pybind11"
        exit 1
    fi
fi

# Check if other dependencies are installed
echo "Checking system dependencies..."

# Check for libserialport
if ! pkg-config --exists libserialport; then
    echo "libserialport not found. Install with:"
    echo "  sudo apt-get install libserialport-dev"
    exit 1
fi

# Check for lcm
if ! pkg-config --exists lcm; then
    echo "lcm not found. Install with:"
    echo "  sudo apt-get install liblcm-dev"
    echo "  or build from source: https://github.com/lcm-proj/lcm"
    exit 1
fi

# Check for yaml-cpp
if ! pkg-config --exists yaml-cpp; then
    echo "yaml-cpp not found. Install with:"
    echo "  sudo apt-get install libyaml-cpp-dev"
    exit 1
fi

echo "All dependencies found!"

# Create build directory
mkdir -p build
cd build

# Run CMake
echo "Running CMake..."
cmake ..
if [ $? -ne 0 ]; then
    echo "CMake configuration failed!"
    exit 1
fi

# Build
echo "Building..."
make -j$(nproc)
if [ $? -ne 0 ]; then
    echo "Build failed!"
    exit 1
fi

echo "Build completed successfully!"

# Check if Python bindings were built
if [ -f "../python/hightorque_robot_py"*.so ]; then
    echo "Python bindings built successfully!"
    echo "You can now run: cd ../python && python3 example_motor_control.py"
else
    echo "Python bindings were not built. Check the CMake output above."
fi