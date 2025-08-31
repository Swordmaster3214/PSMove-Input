cmake_minimum_required(VERSION 3.10)
project(move2uinput LANGUAGES CXX)

# Export compile_commands.json for editor/clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------- Dependencies ----------
find_package(PkgConfig REQUIRED)

# libevdev via pkg-config
pkg_check_modules(LIBEVDEV REQUIRED libevdev)
if(NOT LIBEVDEV_FOUND)
  message(FATAL_ERROR "libevdev not found (install libevdev-dev / libevdev-devel)")
endif()

# Try to find PSMove API library/header (package names vary by distro)
find_path(PSMOVE_INCLUDE_DIR NAMES psmove.h
          HINTS /usr/include /usr/local/include)
find_library(PSMOVE_LIBRARY NAMES psmove psmoveapi psmoveapi_static
             HINTS /usr/lib /usr/local/lib /usr/lib64)

if(PSMOVE_INCLUDE_DIR AND PSMOVE_LIBRARY)
  message(STATUS "Found PSMove: include=${PSMOVE_INCLUDE_DIR}, lib=${PSMOVE_LIBRARY}")
else()
  message(WARNING "PSMove headers/libs not found automatically. "
                  "If you plan to build PSMove integration, install psmoveapi dev package "
                  "or set PSMOVE_INCLUDE_DIR and PSMOVE_LIBRARY manually.")
endif()

# ---------- Sources ----------
file(GLOB_RECURSE SRC_FILES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
    "${CMAKE_SOURCE_DIR}/src/*.cc"
)

if(NOT SRC_FILES)
  message(FATAL_ERROR "No source files found under ${CMAKE_SOURCE_DIR}/src")
endif()

add_executable(${PROJECT_NAME} ${SRC_FILES})

# ---------- Include directories & compile options ----------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${LIBEVDEV_INCLUDE_DIRS}
)

if(PSMOVE_INCLUDE_DIR)
  target_include_directories(${PROJECT_NAME} PRIVATE ${PSMOVE_INCLUDE_DIR})
endif()

# pkg-config libs and other link flags
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBEVDEV_LIBRARIES})

if(PSMOVE_LIBRARY)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${PSMOVE_LIBRARY})
endif()

# pthread (used by threads)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

# Common compile flags
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O2)

# Provide the pkg-config include/link flags to the target for editors
# (pkg_check_modules sets the variables we used above)
if(LIBEVDEV_CFLAGS)
  foreach(flag ${LIBEVDEV_CFLAGS})
    target_compile_options(${PROJECT_NAME} PRIVATE ${flag})
  endforeach()
endif()

# ---------- Feature checks ----------
include(CheckSymbolExists)
# check for the convenience function libevdev_set_id (some versions expose it)
# we need to specify the header to check against
check_symbol_exists(libevdev_set_id "libevdev/libevdev.h" HAVE_LIBEVDEV_SET_ID)
if(HAVE_LIBEVDEV_SET_ID)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_LIBEVDEV_SET_ID=1)
  message(STATUS "libevdev_set_id available: using single-call id setter")
else()
  message(STATUS "libevdev_set_id not available: will use per-field id setters or skip")
endif()

# Another useful check: whether libevdev_set_id_vendor exists (very common)
check_symbol_exists(libevdev_set_id_vendor "libevdev/libevdev.h" HAVE_LIBEVDEV_SET_ID_VENDOR)
if(HAVE_LIBEVDEV_SET_ID_VENDOR)
  target_compile_definitions(${PROJECT_NAME} PRIVATE HAVE_LIBEVDEV_SET_ID_VENDOR=1)
endif()

# ---------- Installation (optional) ----------
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin)

# ---------- Helpful output ----------
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Sources: ${SRC_FILES}")
message(STATUS "libevdev includes: ${LIBEVDEV_INCLUDE_DIRS}")
message(STATUS "libevdev libs: ${LIBEVDEV_LIBRARIES}")
if(PSMOVE_INCLUDE_DIR AND PSMOVE_LIBRARY)
  message(STATUS "PSMove include: ${PSMOVE_INCLUDE_DIR}")
  message(STATUS "PSMove lib: ${PSMOVE_LIBRARY}")
else()
  message(STATUS "PSMove not fully configured — PSMove integration will be disabled/unavailable until headers/libs are found.")
endif()

# ---------- Optional helper variables ----------
option(ENABLE_STRICT_WARNINGS "Enable -Werror" OFF)
if(ENABLE_STRICT_WARNINGS)
  target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
endif()
