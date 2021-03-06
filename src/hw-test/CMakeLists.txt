# HW_TEST_UNIT should always be set to * in 'test-*.cpp' in
# their respective filename that contains main().
if(HW_TEST_UNIT AND NOT(HW_TEST_UNIT MATCHES "default"))
    # Include the arm toolchain for gcc
    set(CMAKE_TOOLCHAIN_FILE ${ARM_TOOLCHAIN_FILE})
    include(${ARM_TOOLCHAIN_FILE})

    # Set compiler and linker flags
    set(CMAKE_CXX_FLAGS         ${MBED_CMAKE_CXX_FLAGS}         )
    set(CMAKE_C_FLAGS           ${MBED_CMAKE_C_FLAGS}           )
    set(CMAKE_EXE_LINKER_FLAGS  ${MBED_CMAKE_EXE_LINKER_FLAGS}  )

    set(ROBOT_HW_TEST_INCLUDES ${CONFIG_ROOT_NAME} ${CMAKE_CURRENT_LIST_DIR})
    set(TEST_UNIT_FILE "test-${HW_TEST_UNIT}.cpp")

    string(SUBSTRING "${HW_TEST_UNIT}" 0 3 TEST_UNIT_BIN_FILE)
    set(TEST_UNIT_BIN_FILE "${TEST_UNIT_BIN_FILE}-test.bin")

    add_executable(robot_test_ELF ${TEST_UNIT_FILE})

    # specify that we depend on the firm-lib library and link to it
    target_link_libraries(robot_test_ELF firm-lib)

    # include the file at test
    target_include_directories(robot_test_ELF PUBLIC ${ROBOT_HW_TEST_INCLUDES})

    # the final product is the .bin file, not the elf one.  We hide this away in the build dir
    set_target_properties(robot_test_ELF PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})

    # only build robot firmware if specifically instructed
    set_target_properties(robot_test_ELF PROPERTIES EXCLUDE_FROM_ALL TRUE)

    # custom target for creating a .bin file from an elf binary
    add_custom_target(robot-test
        ${ARM_OBJCOPY} -O binary robot_test_ELF ${TEST_UNIT_BIN_FILE}
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/${TEST_UNIT_BIN_FILE} ${PROJECT_SOURCE_DIR}/run/
        DEPENDS robot_test_ELF
        COMMENT "Objcopying for making the ${HW_TEST_UNIT} hardware test"
    )
    set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES ${PROJECT_SOURCE_DIR}/run/${TEST_UNIT_BIN_FILE})

    add_custom_target(robot-test-prog
        COMMAND ${MBED_COPY_SCRIPT} ${CMAKE_CURRENT_BINARY_DIR}/${TEST_UNIT_BIN_FILE}
        DEPENDS robot-test
        COMMENT "Copying ${HW_TEST_UNIT}'s test binary over to the mbed"
    )
endif()
