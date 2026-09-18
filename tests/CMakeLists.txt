# Base model always available
set(TEST_MODELS "Kinova")

# Check which components were built by looking for the .so files
find_file(
  _HAS_CAMERA mc_realsense_camera.so
  PATHS "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}"
  NO_DEFAULT_PATH)
find_file(
  _HAS_GRIPPER mc_robotiq_gripper.so
  PATHS "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}"
  NO_DEFAULT_PATH)
find_file(
  _HAS_HANDE mc_robotiq_hande.so
  PATHS "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}"
  NO_DEFAULT_PATH)
find_file(
  _HAS_BOTA mc_bota_sensor.so
  PATHS "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}"
  NO_DEFAULT_PATH)
find_file(
  _HAS_DS4 mc_ds4.so
  PATHS "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}"
  NO_DEFAULT_PATH)
find_file(
  _HAS_PLATE mc_plate.so
  PATHS "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}"
  NO_DEFAULT_PATH)
find_file(
  _HAS_SCREW mc_screw.so
  PATHS "${MC_ROBOTS_RUNTIME_INSTALL_PREFIX}"
  NO_DEFAULT_PATH)

if(_HAS_CAMERA)
  list(APPEND TEST_MODELS "KinovaCamera")
endif()
if(_HAS_GRIPPER)
  list(APPEND TEST_MODELS "KinovaGripper")
endif()
if(_HAS_HANDE)
  list(APPEND TEST_MODELS "KinovaRobotiqHandEGripper")
endif()
if(_HAS_BOTA)
  list(APPEND TEST_MODELS "KinovaBota")
  if(_HAS_DS4)
    list(APPEND TEST_MODELS "KinovaBotaDS4")
  endif()
  if(_HAS_PLATE)
    list(APPEND TEST_MODELS "KinovaBotaPlate")
  endif()
  if(_HAS_SCREW)
    list(APPEND TEST_MODELS "KinovaBotaScrew")
  endif()
endif()

message(STATUS "Tests: will test models: ${TEST_MODELS}")

if(TARGET kinova)
  foreach(MODEL ${TEST_MODELS})
    string(TOLOWER ${MODEL} MODEL_LOWER)
    set(ROBOT_MODULE_PARAMS "\"${MODEL}\"")

    configure_file(
      "${CMAKE_CURRENT_SOURCE_DIR}/loader.in.cpp"
      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/loader_${MODEL_LOWER}.cpp")
    add_executable(
      loader_${MODEL_LOWER}
      "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/loader_${MODEL_LOWER}.cpp")
    target_link_libraries(loader_${MODEL_LOWER} PUBLIC mc_rtc::mc_rbdyn)
    add_test(NAME loader_${MODEL_LOWER} COMMAND loader_${MODEL_LOWER}
                                                "$<TARGET_FILE_DIR:kinova>")
  endforeach()
endif()
