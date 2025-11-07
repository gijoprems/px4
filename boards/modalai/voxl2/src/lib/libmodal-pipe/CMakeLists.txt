cmake_minimum_required(VERSION 3.3)
project(libmodal_pipe)


set(LIBNAME modal_pipe)

# James' standard list of cmake flags
set(CMAKE_C_FLAGS "-std=gnu99 -Wall -Wextra -Wuninitialized \
	-Wunused-variable -Wdouble-promotion -Wmissing-prototypes \
	-Wmissing-declarations -Werror=undef -Wno-unused-function ${CMAKE_C_FLAGS}")


# for VOXL, install 64-bit libraries to lib64, 32-bit libs go in /usr/lib
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^aarch64")
	set(LIB_INSTALL_DIR /usr/lib64)
else()
	set(LIB_INSTALL_DIR /usr/lib)
endif()

if(PLATFORM MATCHES APQ8096)
	add_definitions(-DPLATFORM_APQ8096)
endif()


# add lirbary subdirectory
add_subdirectory(library)

# Don't both with 32-bit examples
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^arm")
	message("skipping examples for arm32")
else()
	add_subdirectory(examples)
endif()


