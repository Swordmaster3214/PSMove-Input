cmake_minimum_required(VERSION 3.16)  # Bumped for better platform support
project(move2uinput LANGUAGES CXX)

# Export compile_commands.json for editor/clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use modern C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Platform detection
message(STATUS "Detected platform: ${CMAKE_SYSTEM_NAME}")

# ---------- Platform-specific Dependencies ----------
set(PLATFORM_LIBS)
set(PLATFORM_SOURCES)
set(PLATFORM_INCLUDES)
set(PLATFORM_DEFINITIONS)

if(WIN32)
    message(STATUS "Configuring for Windows...")
    
    # ViGEm Client SDK (recommended approach)
    find_path(VIGEM_INCLUDE_DIR 
        NAMES ViGEm/Client.h
        HINTS 
            "C:/Program Files/ViGEmBus/SDK/inc"
            "${CMAKE_SOURCE_DIR}/third_party/ViGEmClient/include"
            $ENV{VIGEM_SDK}/include
    )
    
    find_library(VIGEM_LIBRARY 
        NAMES ViGEmClient
        HINTS 
            "C:/Program Files/ViGEmBus/SDK/lib/amd64"
            "C:/Program Files/ViGEmBus/SDK/lib/x86"
            "${CMAKE_SOURCE_DIR}/third_party/ViGEmClient/lib"
            $ENV{VIGEM_SDK}/lib
    )
    
    if(VIGEM_INCLUDE_DIR AND VIGEM_LIBRARY)
        message(STATUS "Found ViGEm: include=${VIGEM_INCLUDE_DIR}, lib=${VIGEM_LIBRARY}")
        set(PLATFORM_INCLUDES ${VIGEM_INCLUDE_DIR})
        set(PLATFORM_LIBS ${VIGEM_LIBRARY})
        set(PLATFORM_DEFINITIONS HAVE_VIGEM=1)
    else()
        # Fallback to vJoy if ViGEm not found
        find_path(VJOY_INCLUDE_DIR 
            NAMES vjoyinterface.h
            HINTS 
                "C:/Program Files/vJoy/SDK/inc"
                "${CMAKE_SOURCE_DIR}/third_party/vJoy/inc"
        )
        
        find_library(VJOY_LIBRARY 
            NAMES vJoyInterface
            HINTS 
                "C:/Program Files/vJoy/SDK/lib"
                "${CMAKE_SOURCE_DIR}/third_party/vJoy/lib"
        )
        
        if(VJOY_INCLUDE_DIR AND VJOY_LIBRARY)
            message(STATUS "Found vJoy: include=${VJOY_INCLUDE_DIR}, lib=${VJOY_LIBRARY}")
            set(PLATFORM_INCLUDES ${VJOY_INCLUDE_DIR})
            set(PLATFORM_LIBS ${VJOY_LIBRARY})
            set(PLATFORM_DEFINITIONS HAVE_VJOY=1)
        else()
            message(WARNING "Neither ViGEm nor vJoy found. Virtual gamepad functionality will be disabled.")
            message(WARNING "Install ViGEm Bus Driver from: https://github.com/ViGEm/ViGEmBus/releases")
            message(WARNING "Or vJoy from: http://vjoystick.sourceforge.net/")
        endif()
    endif()
    
    # Windows system libraries
    set(PLATFORM_LIBS ${PLATFORM_LIBS} setupapi kernel32 user32)
    
    # Windows-specific sources
    file(GLOB WIN_SOURCES "${CMAKE_SOURCE_DIR}/src/windows/*.cpp")
    set(PLATFORM_SOURCES ${WIN_SOURCES})
    
elseif(UNIX AND NOT APPLE)
    message(STATUS "Configuring for Linux...")
    
    # libevdev via pkg-config (Linux only)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBEVDEV REQUIRED libevdev)
    if(NOT LIBEVDEV_FOUND)
        message(FATAL_ERROR "libevdev not found (install libevdev-dev / libevdev-devel)")
    endif()
    
    set(PLATFORM_INCLUDES ${LIBEVDEV_INCLUDE_DIRS})
    set(PLATFORM_LIBS ${LIBEVDEV_LIBRARIES})
    set(PLATFORM_DEFINITIONS HAVE_UINPUT=1)
    
    # Linux-specific sources
    file(GLOB LINUX_SOURCES "${CMAKE_SOURCE_DIR}/src/linux/*.cpp")
    set(PLATFORM_SOURCES ${LINUX_SOURCES})
    
    # Add libevdev compile flags
    if(LIBEVDEV_CFLAGS)
        foreach(flag ${LIBEVDEV_CFLAGS})
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}")
        endforeach()
    endif()
    
    # Feature checks for libevdev
    include(CheckSymbolExists)
    check_symbol_exists(libevdev_set_id "libevdev/libevdev.h" HAVE_LIBEVDEV_SET_ID)
    if(HAVE_LIBEVDEV_SET_ID)
        set(PLATFORM_DEFINITIONS ${PLATFORM_DEFINITIONS} HAVE_LIBEVDEV_SET_ID=1)
        message(STATUS "libevdev_set_id available: using single-call id setter")
    endif()
    
    check_symbol_exists(libevdev_set_id_vendor "libevdev/libevdev.h" HAVE_LIBEVDEV_SET_ID_VENDOR)
    if(HAVE_LIBEVDEV_SET_ID_VENDOR)
        set(PLATFORM_DEFINITIONS ${PLATFORM_DEFINITIONS} HAVE_LIBEVDEV_SET_ID_VENDOR=1)
    endif()
    
elseif(APPLE)
    message(WARNING "macOS support not implemented yet")
    # TODO: Add macOS virtual gamepad support (IOKit framework?)
    
else()
    message(FATAL_ERROR "Unsupported platform: ${CMAKE_SYSTEM_NAME}")
endif()

# ---------- Cross-platform PSMove dependency ----------
if(WIN32)
    # Windows: PSMove API might be in different locations
    find_path(PSMOVE_INCLUDE_DIR 
        NAMES psmove.h
        HINTS 
            "${CMAKE_SOURCE_DIR}/third_party/psmoveapi/include"
            "C:/psmoveapi/include"
            $ENV{PSMOVE_DIR}/include
    )
    find_library(PSMOVE_LIBRARY 
        NAMES psmove psmoveapi
        HINTS 
            "${CMAKE_SOURCE_DIR}/third_party/psmoveapi/lib"
            "C:/psmoveapi/lib"
            $ENV{PSMOVE_DIR}/lib
    )
else()
    # Linux: Use pkg-config first, then fallback to find_path/find_library
    if(PkgConfig_FOUND)
        pkg_check_modules(PSMOVE psmoveapi)
    endif()
    
    if(NOT PSMOVE_FOUND)
        find_path(PSMOVE_INCLUDE_DIR 
            NAMES psmove.h
            HINTS /usr/include /usr/local/include
        )
        find_library(PSMOVE_LIBRARY 
            NAMES psmove psmoveapi psmoveapi_static
            HINTS /usr/lib /usr/local/lib /usr/lib64
        )
    endif()
endif()

if(PSMOVE_INCLUDE_DIR AND PSMOVE_LIBRARY)
    message(STATUS "Found PSMove: include=${PSMOVE_INCLUDE_DIR}, lib=${PSMOVE_LIBRARY}")
    set(PLATFORM_INCLUDES ${PLATFORM_INCLUDES} ${PSMOVE_INCLUDE_DIR})
    set(PLATFORM_LIBS ${PLATFORM_LIBS} ${PSMOVE_LIBRARY})
    set(PLATFORM_DEFINITIONS ${PLATFORM_DEFINITIONS} HAVE_PSMOVE=1)
elseif(PSMOVE_FOUND)
    message(STATUS "Found PSMove via pkg-config")
    set(PLATFORM_INCLUDES ${PLATFORM_INCLUDES} ${PSMOVE_INCLUDE_DIRS})
    set(PLATFORM_LIBS ${PLATFORM_LIBS} ${PSMOVE_LIBRARIES})
    set(PLATFORM_DEFINITIONS ${PLATFORM_DEFINITIONS} HAVE_PSMOVE=1)
else()
    message(WARNING "PSMove API not found. Controller polling will be disabled.")
    message(WARNING "Install psmoveapi from: https://github.com/thp/psmoveapi")
endif()

# ---------- Sources ----------
# Core cross-platform sources
file(GLOB CORE_SOURCES
    "${CMAKE_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/src/*.c"
    "${CMAKE_SOURCE_DIR}/src/*.cc"
)

# Combine core and platform-specific sources
set(ALL_SOURCES ${CORE_SOURCES} ${PLATFORM_SOURCES})

if(NOT ALL_SOURCES)
    message(FATAL_ERROR "No source files found under ${CMAKE_SOURCE_DIR}/src")
endif()

add_executable(${PROJECT_NAME} ${ALL_SOURCES})

# ---------- Include directories & compile definitions ----------
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${PLATFORM_INCLUDES}
)

if(PLATFORM_DEFINITIONS)
    target_compile_definitions(${PROJECT_NAME} PRIVATE ${PLATFORM_DEFINITIONS})
endif()

# ---------- Linking ----------
target_link_libraries(${PROJECT_NAME} PRIVATE ${PLATFORM_LIBS})

# Threading support (cross-platform)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

# ---------- Compiler flags ----------
if(MSVC)
    # Visual Studio specific flags
    target_compile_options(${PROJECT_NAME} PRIVATE 
        /W4          # High warning level
        /O2          # Optimize for speed
        /DWIN32_LEAN_AND_MEAN
        /DNOMINMAX   # Prevent min/max macro conflicts
    )
else()
    # GCC/Clang flags
    target_compile_options(${PROJECT_NAME} PRIVATE 
        -Wall -Wextra -O2
    )
endif()

# ---------- Optional strict warnings ----------
option(ENABLE_STRICT_WARNINGS "Enable -Werror (or /WX on MSVC)" OFF)
if(ENABLE_STRICT_WARNINGS)
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /WX)
    else()
        target_compile_options(${PROJECT_NAME} PRIVATE -Werror)
    endif()
endif()

# ---------- Installation ----------
install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin)

# On Windows, also install required DLLs if found
if(WIN32)
    # Try to find and install ViGEm/vJoy DLLs
    if(VIGEM_LIBRARY)
        get_filename_component(VIGEM_DIR ${VIGEM_LIBRARY} DIRECTORY)
        file(GLOB VIGEM_DLLS "${VIGEM_DIR}/*.dll")
        if(VIGEM_DLLS)
            install(FILES ${VIGEM_DLLS} DESTINATION bin)
        endif()
    endif()
endif()

# ---------- Helpful output ----------
message(STATUS "=== Build Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Platform: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Sources: ${ALL_SOURCES}")
message(STATUS "Platform includes: ${PLATFORM_INCLUDES}")
message(STATUS "Platform libraries: ${PLATFORM_LIBS}")
message(STATUS "Platform definitions: ${PLATFORM_DEFINITIONS}")

if(WIN32)
    if(VIGEM_INCLUDE_DIR AND VIGEM_LIBRARY)
        message(STATUS "Virtual gamepad: ViGEm")
    elseif(VJOY_INCLUDE_DIR AND VJOY_LIBRARY)
        message(STATUS "Virtual gamepad: vJoy")
    else()
        message(STATUS "Virtual gamepad: NONE (install ViGEm or vJoy)")
    endif()
else()
    message(STATUS "Virtual gamepad: uinput (Linux)")
endif()

if(PSMOVE_INCLUDE_DIR OR PSMOVE_FOUND)
    message(STATUS "PSMove support: YES")
else()
    message(STATUS "PSMove support: NO (install psmoveapi)")
endif()

message(STATUS "===========================")

# ---------- Usage hints ----------
if(WIN32)
    message(STATUS "")
    message(STATUS "Windows build notes:")
    message(STATUS "  - Install ViGEm Bus Driver: https://github.com/ViGEm/ViGEmBus/releases")
    message(STATUS "  - Or install vJoy: http://vjoystick.sourceforge.net/")
    message(STATUS "  - Build PSMove API: https://github.com/thp/psmoveapi")
    message(STATUS "  - Run as Administrator if needed for virtual device creation")
endif()
