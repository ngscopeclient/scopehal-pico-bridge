
if(WIN32)
	# Windows additional options, libraries and default path (Picoscope SDK 64bits)
	# Use MinGW formatting (C *printf Format "%zu"...) rather than MSVCRT for C99 support.
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D__USE_MINGW_ANSI_STDIO=1")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__USE_MINGW_ANSI_STDIO=1")
	set(WIN_LIBS shlwapi)
	set(PICO_SDK_PATH "C:/Program Files/Pico Technology/SDK")
elseif(APPLE)
	find_library(PICO_SDK_FRAMEWORK PicoSDK)
	if (NOT PICO_SDK_FRAMEWORK)
		message(FATAL_ERROR PicoSDK not installed)
	endif()
else()
	# Linux specific Picoscope SDK Path
	set(PICO_SDK_PATH "/opt/picoscope")
endif()

#Set up include paths
if(WIN32)
	# Windows specific include path
	include_directories(${PICO_SDK_PATH}/inc/)
elseif(APPLE)
	include_directories(${PICO_SDK_FRAMEWORK}/Headers/libps3000a)
	include_directories(${PICO_SDK_FRAMEWORK}/Headers/libps6000a)
else()
	# Linux specific include paths
	include_directories(${PICO_SDK_PATH}/include/libps3000a)
	include_directories(${PICO_SDK_PATH}/include/libps6000a)
endif()

###############################################################################
#C++ compilation
add_executable(ps6000d
	PicoSCPIServer.cpp
	WaveformServerThread.cpp
	main.cpp
)

###############################################################################
#Linker settings
if(WIN32)
	# Windows specific linker
	target_link_libraries(ps6000d
	xptools
	log
	scpi-server-tools
	${PICO_SDK_PATH}/lib/ps3000a.lib
	${PICO_SDK_PATH}/lib/ps6000a.lib
	)
elseif(APPLE)
	target_link_libraries(ps6000d
	xptools
	log
	scpi-server-tools
	${PICO_SDK_FRAMEWORK}/Libraries/libps3000a/libps3000a.dylib
	${PICO_SDK_FRAMEWORK}/Libraries/libps6000a/libps6000a.dylib)
else()
	# Linux specific linker
	target_link_libraries(ps6000d
	xptools
	log
	scpi-server-tools
	${PICO_SDK_PATH}/lib/libps3000a.so
	${PICO_SDK_PATH}/lib/libps6000a.so
	)
endif()

