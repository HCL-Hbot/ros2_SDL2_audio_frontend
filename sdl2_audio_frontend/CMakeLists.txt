cmake_minimum_required(VERSION 3.8)
project(sdl2_audio_frontend)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add cmake modules directory
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# find dependencies
find_package(PkgConfig REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(SDL2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AudioData.msg"
  DEPENDENCIES std_msgs
)

# Need to link message interfaces
rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# SDL2 Audio utilities library
add_library(sdl2_audio_utils
  src/audio/audio_async.cpp
)
target_include_directories(sdl2_audio_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(sdl2_audio_utils PUBLIC
  ${SDL2_LIBRARIES}
)
target_compile_features(sdl2_audio_utils PUBLIC cxx_std_17)

# Fetch Lightweight openwakeword implementation 
set (LOWWI_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/Lowwi) 
link_directories(${LOWWI_PATH}/lib/x64) # Add the ONNX runtime library paths
link_directories(${LOWWI_PATH}/lib/x64/lib)
include(FetchContent)
FetchContent_Declare(
  Lowwi
  GIT_REPOSITORY https://github.com/CLFML/lowwi
  GIT_TAG main
  SOURCE_DIR ${LOWWI_PATH}
)
FetchContent_MakeAvailable(Lowwi)

# Audio Capture Node
add_executable(audio_capture_node src/nodes/audio_capture_node.cpp)
target_include_directories(audio_capture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(audio_capture_node PUBLIC cxx_std_17)

target_link_libraries(audio_capture_node
  sdl2_audio_utils
  ${SDL2_LIBRARIES}  
  "${cpp_typesupport_target}"
)

ament_target_dependencies(audio_capture_node
  rclcpp
  std_msgs
)

# VAD Node
add_executable(vad_node src/nodes/vad_node.cpp)
target_include_directories(vad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(vad_node PUBLIC cxx_std_17)
target_link_libraries(vad_node
  sdl2_audio_utils
  "${cpp_typesupport_target}"
)
ament_target_dependencies(vad_node
  rclcpp
  std_msgs
)

# Wake Word Node
add_executable(wake_word_node src/nodes/wake_word_node.cpp)
message(STATUS "Lowwi source dir: ${Lowwi_SOURCE_DIR}")
target_include_directories(wake_word_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Lowwi_SOURCE_DIR}/src  # Lowwi headers
)

target_compile_features(wake_word_node PUBLIC cxx_std_17)
target_link_libraries(wake_word_node
  sdl2_audio_utils
  Lowwi  # Link with Lowwi library
  "${cpp_typesupport_target}"
  ncurses
)

ament_target_dependencies(wake_word_node
  rclcpp
  std_msgs
  ament_index_cpp
)

# Install libraries
install(TARGETS
  sdl2_audio_utils
  Lowwi
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY 
    "${Lowwi_SOURCE_DIR}/models"
    DESTINATION share/${PROJECT_NAME}
)

# # Custom command to copy models during build (for development)
# add_custom_target(copy_Lowwi_models ALL
#     COMMAND ${CMAKE_COMMAND} -E copy_directory
#         ${Lowwi_SOURCE_DIR}/models
#         ${CMAKE_CURRENT_BINARY_DIR}/models
#     COMMENT "Copying models directory to build folder"
# )

# Install ONNX Runtime from Lowwi's lib directory
install(FILES 
    "${LOWWI_PATH}/lib/x64/lib/libonnxruntime.so"
    "${LOWWI_PATH}/lib/x64/lib/libonnxruntime.so.1.18.0"
    DESTINATION lib
)

# Install targets
install(TARGETS
  audio_capture_node
  vad_node
  wake_word_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()