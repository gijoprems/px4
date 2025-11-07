
# Build lib from all source files
file(GLOB all_src_files src/*.c)
add_library(${LIBNAME} SHARED ${all_src_files})

# for VOXL, install 64-bit libraries to lib64, 32-bit libs go in /usr/lib
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
	target_link_libraries(${LIBNAME} LINK_PUBLIC pthread /usr/lib64/libmodal_json.so)
else()
	target_link_libraries(${LIBNAME} LINK_PUBLIC pthread modal_json)
endif()


target_include_directories(${LIBNAME} PUBLIC include )
target_include_directories(${LIBNAME} PUBLIC .) # for mavlink folder

# make the include directory public for install
file(GLOB LIB_HEADERS include/*.h)
set_target_properties(${LIBNAME} PROPERTIES PUBLIC_HEADER "${LIB_HEADERS}")

# make sure everything is installed where we want
# LIB_INSTALL_DIR comes from the parent cmake file
install(
	TARGETS			${LIBNAME}
	LIBRARY			DESTINATION ${LIB_INSTALL_DIR}
	RUNTIME			DESTINATION /usr/bin
	PUBLIC_HEADER	DESTINATION /usr/include
)
