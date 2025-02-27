find_package(Git)

if(GIT_EXECUTABLE)
	get_filename_component(WORKING_DIR ${SRC} DIRECTORY)
	execute_process(
		COMMAND ${GIT_EXECUTABLE} --git-dir ${WORKING_DIR}/.git describe --tags --dirty
		OUTPUT_VARIABLE M33_FW_VERSION
		RESULT_VARIABLE ERROR_CODE
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
endif()

if(M33_FW_VERSION STREQUAL "")
	set(M33_FW_VERSION 0.0.0-no-git)
	message(WARNING "Failed to determine version from Git tags. Using default version \"${M33_FW_VERSION}\".")
endif()

configure_file(${SRC} ${DST} @ONLY)
