ament_add_test(test_backup_behavior
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_backup_behavior.launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 60
  ENV
    TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
)
