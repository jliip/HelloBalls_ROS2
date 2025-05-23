#!/bin/bash

echo "Setting up YOLO11 detector dependencies..."

# Get the absolute path to the workspace
WORKSPACE_DIR="/home/sunrise/Documents/ros2_ws"
YOLO_DIR="$WORKSPACE_DIR/lib/yolo11_demo"
CPP_DIR="$YOLO_DIR/cpp"
BUILD_DIR="$CPP_DIR/build"

# Check if build directory exists
if [ ! -d "$BUILD_DIR" ]; then
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
fi

# Build the YOLO11 API
echo "Building YOLO11 API..."
cd "$CPP_DIR"
cd build
cmake .. -DMODEL_PATH="$YOLO_DIR/ptq_models/yolo11m_detect_bayese_640x640_nv12_modified.bin"
make -j4

# Check if the build was successful
if [ $? -eq 0 ]; then
    echo "Build successful!"
    
    # Copy the module to the Python path for easier access
    echo "Copying module to ROS package directory..."
    PACKAGE_DIR="$WORKSPACE_DIR/src/yolo11_detector/yolo11_detector"
    cp "$BUILD_DIR"/yolo11_api*.so "$PACKAGE_DIR"
    
    echo "Setup complete. You can now run the YOLO11 detector node."
else
    echo "Build failed. Please check the error messages."
    exit 1
fi