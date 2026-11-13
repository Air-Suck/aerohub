# Isolated harness for the actual patched platform class; no PX4 is connected.
add_executable(ipc_pairing_test /repo/deployment/airsim/demos/ipc_control/tests/pairing.cpp)
ament_target_dependencies(ipc_pairing_test ${PROJECT_DEPENDENCIES})
target_link_libraries(ipc_pairing_test ${PROJECT_NAME})
