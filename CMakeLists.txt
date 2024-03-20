cmake_minimum_required(VERSION 3.3)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
project(scopehal-pico-bridge)

set(ANALYZE CACHE BOOL "Run static analysis on the code, requires cppcheck and clang-analyzer to be installed")

set(WARNINGS "-Wall -Wextra -Wuninitialized ")
set(WARNINGS "${WARNINGS} -Wshadow -Wunsafe-loop-optimizations -Wpedantic -Wcast-align -Wwrite-strings")
set(WARNINGS "${WARNINGS} -Wmissing-declarations -Wvla")
set(CMAKE_CXX_FLAGS "-g -fopenmp ${WARNINGS} --std=c++11 -ffast-math")
if(NOT (APPLE AND ${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "arm64"))
	message(${CMAKE_HOST_SYSTEM_PROCESSOR})
	# we can not build for arm64, but mtune=native will still somehow resolve to apple-m1 or similar,
	# which x86 apple clang does not understand.
	# Only set mtune=native if we're *not* building on an Apple Silicon host.
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mtune=native")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -fsanitize=address")

if(APPLE AND ${CMAKE_HOST_SYSTEM_PROCESSOR} STREQUAL "arm64")
	# force x86 build, PicoSDK is only built for Intel macs as of 10.7.26.362, so arm64 can't link against it
	set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE INTERNAL "" FORCE)
endif ()

if(WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_USE_MATH_DEFINES -D_POSIX_THREAD_SAFE_FUNCTIONS")
endif()

find_package(PkgConfig)
find_package(OpenCL)

if(ANALYZE)
	find_program(CPPCHECK_PATH cppcheck DOC "Path to cppcheck when ANALYZE is enabled")
	if(CPPCHECK_PATH)
		execute_process(COMMAND ${CPPCHECK_PATH} "--version" OUTPUT_VARIABLE CPPCHECK_VER_STR ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
		string(REPLACE "Cppcheck " "" CPPCHECK_VERSION ${CPPCHECK_VER_STR})
		if(CPPCHECK_VERSION VERSION_GREATER_EQUAL "2")
			set(CMAKE_CXX_CPPCHECK "${CPPCHECK_PATH};-DFT_USE_AUTOCONF_SIZEOF_TYPES;-D__GNUC__;--enable=warning,performance,portability;--suppress=*:*sigc*;--suppress=*:*glibmm*;--suppress=*:*gtkmm*;--inline-suppr;-q;--std=c++11")
			message(STATUS "Found CPPCheck: ${CPPCHECK_PATH} (found version \"${CPPCHECK_VERSION}\")")
		else()
			message(STATUS "Found CPPCheck: ${CPPCHECK_PATH} but ignored it as it was ${CPPCHECK_VERSION} < 2")
		endif()
	else()
		message(STATUS "CPPCheck not found")
	endif()
	# The actual clang-analyzer compiler wrapper doesn't get installed on $PATH, only scan-build which is useless to us
	find_program(CLANGANALYZER_SCANBUILD_PATH scan-build DOC "Path to clang-analyzer's scan-build tool, used as a hint to find the rest of the clang-analyzer")
	get_filename_component(CLANGANALYZER_SCANBUILD_BIN ${CLANGANALYZER_SCANBUILD_PATH} REALPATH)
	get_filename_component(CLANGANALYZER_BIN_PATH ${CLANGANALYZER_SCANBUILD_BIN} DIRECTORY)
	find_program(CLANGANALYZER_CXXANALYZER_PATH "c++-analyzer" HINTS "${CLANGANALYZER_BIN_PATH}/../libexec" DOC "Path to clang-analyzer's c++-analyzer")
	if(CLANGANALYZER_CXXANALYZER_PATH)
		set(CMAKE_CXX_COMPILER_LAUNCHER "${CLANGANALYZER_CXXANALYZER_PATH}")
		message(STATUS "Found clang-analyzer: ${CLANGANALYZER_CXXANALYZER_PATH}")
	else()
		message(STATUS "clang-analyzer not found")
	endif()
endif()

add_subdirectory("${PROJECT_SOURCE_DIR}/lib/log")
add_subdirectory("${PROJECT_SOURCE_DIR}/lib/scpi-server-tools")
add_subdirectory("${PROJECT_SOURCE_DIR}/lib/xptools")
add_subdirectory("${PROJECT_SOURCE_DIR}/src/ps6000d")
