cmake_minimum_required(VERSION 3.8)
project(robomaster_gazebo)
find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(AWS_SMALL_HOUSE_WORLD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/aws_robomaker_small_house_world)

# Upstream AWS models often omit (or zero) inertia tensors. Classic Gazebo
# ignored that for static scenery; Ignition aborts world load. Keep the
# submodule pristine and install a patched copy instead.
set(AWS_MODELS_PATCHED "${CMAKE_CURRENT_BINARY_DIR}/aws_models_patched")
file(REMOVE_RECURSE "${AWS_MODELS_PATCHED}")
file(COPY "${AWS_SMALL_HOUSE_WORLD_DIR}/models/" DESTINATION "${AWS_MODELS_PATCHED}")
execute_process(
  COMMAND
    "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/patch_aws_model_inertias.py"
    "${AWS_MODELS_PATCHED}"
  RESULT_VARIABLE _aws_inertia_patch_result
  OUTPUT_VARIABLE _aws_inertia_patch_out
  ERROR_VARIABLE _aws_inertia_patch_err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
if(NOT _aws_inertia_patch_result EQUAL 0)
  message(FATAL_ERROR
    "Failed to patch AWS house model inertias for Ignition:\n"
    "${_aws_inertia_patch_err}\n${_aws_inertia_patch_out}")
endif()
message(STATUS "${_aws_inertia_patch_out}")

install(DIRECTORY launch worlds models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY "${AWS_MODELS_PATCHED}/" DESTINATION share/${PROJECT_NAME})
install(FILES ${AWS_SMALL_HOUSE_WORLD_DIR}/worlds/small_house.world DESTINATION share/${PROJECT_NAME}/worlds)
ament_package()
