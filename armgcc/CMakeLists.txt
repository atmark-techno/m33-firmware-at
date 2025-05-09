# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)

# THE VERSION NUMBER
SET (MCUXPRESSO_CMAKE_FORMAT_MAJOR_VERSION 2)
SET (MCUXPRESSO_CMAKE_FORMAT_MINOR_VERSION 0)

include(ide_overrides.cmake OPTIONAL)

if(CMAKE_SCRIPT_MODE_FILE)
  message("${MCUXPRESSO_CMAKE_FORMAT_MAJOR_VERSION}")
  return()
endif()


# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

if (DEFINED ENV{SdkRootDirPath})
    SET(SdkRootDirPath "$ENV{SdkRootDirPath}")
else()
    message(FATAL_ERROR "SdkRootDirPath is not defined")
endif()

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(m33_firmware_at)

set(MCUX_BUILD_TYPES debug release flash_debug flash_release)

set(MCUX_SDK_PROJECT_NAME m33-firmware-at.elf)

if (NOT DEFINED SdkRootDirPath)
    SET(SdkRootDirPath ${SdkRootDirPath})
endif()

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../rsc_table.c"
"${ProjDirPath}/../rsc_table.h"
"${ProjDirPath}/../remoteproc.h"
"${ProjDirPath}/../rpmsg_config.h"
"${ProjDirPath}/../srtm_config.h"
"${ProjDirPath}/../main.c"
"${ProjDirPath}/../main.h"
"${ProjDirPath}/../lpm.c"
"${ProjDirPath}/../lpm.h"
"${ProjDirPath}/../FreeRTOSConfig.h"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../boards/a900/board.c"
"${ProjDirPath}/../boards/a900/board.h"
"${ProjDirPath}/../boards/a900/clock_config.c"
"${ProjDirPath}/../boards/a900/clock_config.h"
"${ProjDirPath}/../app_srtm.c"
"${ProjDirPath}/../app_srtm.h"
"${ProjDirPath}/../app_srtm_internal.h"
"${ProjDirPath}/../app_adc.c"
"${ProjDirPath}/../app_i2c.c"
"${ProjDirPath}/../app_pwm.c"
"${ProjDirPath}/../app_uboot.h"
"${ProjDirPath}/../app_spi.c"
"${ProjDirPath}/../spi_bitbang_txrx.h"
"${ProjDirPath}/../app_tty.c"
"${ProjDirPath}/../app_tty.h"
"${ProjDirPath}/../app_tty_lpuart.c"
"${ProjDirPath}/../app_tty_flexio.c"
"${ProjDirPath}/../custom/app_tty_custom.c"
"${ProjDirPath}/../app_tty_console.c"
"${ProjDirPath}/../app_tty_console.h"
"${ProjDirPath}/../app_can.c"
"${ProjDirPath}/../app_can.h"
"${ProjDirPath}/../can.h"
"${ProjDirPath}/../can_netlink.h"
"${ProjDirPath}/../lpuart.c"
"${ProjDirPath}/../lpuart.h"
"${ProjDirPath}/../termbits.h"
"${ProjDirPath}/../tty.c"
"${ProjDirPath}/../tty.h"
"${ProjDirPath}/../debug_console.c"
"${ProjDirPath}/../debug_console.h"
"${ProjDirPath}/../3rdparty/printf/printf.c"
"${ProjDirPath}/../3rdparty/printf/printf.h"
"${ProjDirPath}/../3rdparty/printf/printf_config.h"
"${ProjDirPath}/../cli.c"
"${ProjDirPath}/../cli.h"
"${ProjDirPath}/../custom/cli_custom.c"
"${ProjDirPath}/../3rdparty/EmbeddedCLI/embedded_cli.c"
"${ProjDirPath}/../3rdparty/EmbeddedCLI/embedded_cli.h"
"${ProjDirPath}/../srtm/channels/srtm_rpmsg_endpoint.h"
"${ProjDirPath}/../srtm/channels/srtm_rpmsg_endpoint.c"
"${ProjDirPath}/../srtm/services/srtm_adc_service.c"
"${ProjDirPath}/../srtm/services/srtm_adc_service.h"
"${ProjDirPath}/../srtm/services/srtm_can_service.c"
"${ProjDirPath}/../srtm/services/srtm_can_service.h"
"${ProjDirPath}/../srtm/services/srtm_i2c_service.c"
"${ProjDirPath}/../srtm/services/srtm_i2c_service.h"
"${ProjDirPath}/../srtm/services/srtm_io_service.c"
"${ProjDirPath}/../srtm/services/srtm_io_service.h"
"${ProjDirPath}/../srtm/services/srtm_lfcl_service.c"
"${ProjDirPath}/../srtm/services/srtm_lfcl_service.h"
"${ProjDirPath}/../srtm/services/srtm_pwm_adapter.c"
"${ProjDirPath}/../srtm/services/srtm_pwm_adapter.h"
"${ProjDirPath}/../srtm/services/srtm_pwm_service.c"
"${ProjDirPath}/../srtm/services/srtm_pwm_service.h"
"${ProjDirPath}/../srtm/services/srtm_spi_service.c"
"${ProjDirPath}/../srtm/services/srtm_spi_service.h"
"${ProjDirPath}/../srtm/services/srtm_tty_service.c"
"${ProjDirPath}/../srtm/services/srtm_tty_service.h"
"${ProjDirPath}/../srtm/services/srtm_wdog_service.c"
"${ProjDirPath}/../srtm/services/srtm_wdog_service.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/..
    ${ProjDirPath}/../3rdparty/printf
    ${SdkRootDirPath}/examples/evkmimx8ulp
    ${ProjDirPath}/../boards/a900
    ${ProjDirPath}/../srtm/channels
    ${ProjDirPath}/../srtm/services
    ${CMAKE_BINARY_DIR}/CMakeFiles/
)

set_source_files_properties("${ProjDirPath}/../FreeRTOSConfig.h" PROPERTIES COMPONENT_CONFIG_FILE "middleware_freertos-kernel_template")

include(${SdkRootDirPath}/core/devices/MIMX8UD5/all_lib_device.cmake)

IF(NOT DEFINED TARGET_LINK_SYSTEM_LIBRARIES)  
    SET(TARGET_LINK_SYSTEM_LIBRARIES "-lm -lc -lgcc -lnosys")  
ENDIF()  

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE ${TARGET_LINK_SYSTEM_LIBRARIES})

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)

ADD_CUSTOM_COMMAND(TARGET ${MCUX_SDK_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY}
	-Obinary ${EXECUTABLE_OUTPUT_PATH}/${MCUX_SDK_PROJECT_NAME} ${EXECUTABLE_OUTPUT_PATH}/m33-firmware-at.bin)

set_target_properties(${MCUX_SDK_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES "output.map;${EXECUTABLE_OUTPUT_PATH}/m33-firmware-at.bin")

# generate version
add_custom_target(version
	${CMAKE_COMMAND} -D SRC=${ProjDirPath}/../version.h.in
	-D DST=${CMAKE_BINARY_DIR}/CMakeFiles/version.h
		-P ${CMAKE_SOURCE_DIR}/generate_version.cmake
)
add_dependencies(${MCUX_SDK_PROJECT_NAME} version)
