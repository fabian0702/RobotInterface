#!/bin/bash

# Generate protobuf and gRPC python 3 files from proto files.
# Copyright (C) BFH roboticsLab
# All rights reserved

# Abort on errors, unset variables or errors in pipes
set -eu -o pipefail

# List of directories to search for .proto files
proto_paths=(
  ./
)

# List of files to generate code for
proto_files=(
  ch/bfh/roboticsLab/Base.proto

  ch/bfh/roboticsLab/robot/RobotControl.proto

  ch/bfh/roboticsLab/task/TaskList.proto
  ch/bfh/roboticsLab/task/Teach.proto

  ch/bfh/roboticsLab/vision/Vision.proto
)

pythonOutputDir=out/python
mkdir -p $pythonOutputDir

cppOutputDir=out/cpp
mkdir -p $cppOutputDir

# Generate files, prefixing -I to each element of proto_paths
python -m grpc_tools.protoc "${proto_paths[@]/#/-I}" \
 --python_out=$pythonOutputDir --grpc_python_out=$pythonOutputDir \
 --cpp_out=$cppOutputDir \
 "${proto_files[@]}"

# --plugin=protoc-gen-cpp="/opt/local/bin/grpc_cpp_plugin" 
#  --plugin=protoc-gen-grpc_cpp="/opt/local/bin/grpc_cpp_plugin" --grpc_cpp_out=$cppOutputDir \
