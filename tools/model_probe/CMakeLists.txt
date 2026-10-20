cmake_minimum_required(VERSION 3.10)
project(rlsim_model_probe LANGUAGES CXX)

# Independent of catkin, LibTorch, robot SDKs and Python packages.
set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_LIST_DIR}/../../third/onnxruntime-linux-x64-1.20.1"
    CACHE PATH "ONNX Runtime installation")
find_path(ORT_INCLUDE onnxruntime_cxx_api.h PATHS "${ONNXRUNTIME_ROOT}/include" NO_DEFAULT_PATH)
find_library(ORT_LIBRARY onnxruntime PATHS "${ONNXRUNTIME_ROOT}/lib" NO_DEFAULT_PATH)
if(NOT ORT_INCLUDE OR NOT ORT_LIBRARY)
    message(FATAL_ERROR "Set ONNXRUNTIME_ROOT to an installation containing include/ and lib/")
endif()
add_executable(rlsim_model_probe main.cpp)
target_compile_features(rlsim_model_probe PRIVATE cxx_std_14)
target_compile_options(rlsim_model_probe PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(rlsim_model_probe SYSTEM PRIVATE "${ORT_INCLUDE}")
target_link_libraries(rlsim_model_probe PRIVATE "${ORT_LIBRARY}")
