# Additional libraries on Windows
if(WIN32)
	set(WIN_LIBS shlwapi)
	endif()

#Set up include paths
include_directories('/opt/picoscope/include/libps6000a')

###############################################################################
#C++ compilation
add_executable(ps6000d
	main.cpp
)

###############################################################################
#Linker settings
target_link_libraries(ps6000d
	xptools
	log
	/opt/picoscope/lib/libps6000a.so
	)
