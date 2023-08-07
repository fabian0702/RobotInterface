#!/bin/bash

# Generate protobuf and gRPC python 3 files from proto files.
# Copyright (C) BFH roboticsLab
# All rights reserved
# Dependencies grpcio grpcio-tools mypy-protobuf

# Abort on errors, unset variables or errors in pipes
set -eu -o pipefail

# List of directories to search for .proto files
proto_paths=(
  ./Protocol
)

# List of files to generate code for
proto_files=(
  ch/bfh/roboticsLab/Base.proto
  ch/bfh/roboticsLab/robot/RobotControl.proto
  ch/bfh/roboticsLab/task/TaskList.proto
  ch/bfh/roboticsLab/task/Teach.proto
  ch/bfh/roboticsLab/vision/Vision.proto
)

outputDir=.

# Generate files, prefixing -I to each element of proto_paths
python -m grpc_tools.protoc "${proto_paths[@]/#/-I}" --python_out=$outputDir --grpc_python_out=$outputDir "${proto_files[@]}"
