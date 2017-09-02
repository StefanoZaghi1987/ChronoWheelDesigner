#--------------------------------------------------------------
# 
# Example of CMake configuration file to build an external 
# project depending on Chrono and on optional Chrono modules.
# 
# This minimal sample project can be used as a template for a
# user project.  Modify sections 1, 2, and 3 below as appropriate.
# 
#--------------------------------------------------------------

cmake_minimum_required(VERSION 2.8)

#-----------------------------------------------------------------------------
# Prevent in-source builds.
#-----------------------------------------------------------------------------

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message( FATAL_ERROR  "In-source build is not possible and not recommended. Choose an empty directory for build output.")
endif(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})

enable_testing()
set(VALGRIND_COMMAND "/usr/bin/valgrind")
set(VALGRIND_COMMAND_OPTIONS "-v --tool=memcheck --leak-check=full --track-fds=yes --num-callers=50 --show-reachable=yes --track-origins=yes --malloc-fill=0xff --free-fill=0xfe")
set(MEMORYCHECK_COMMAND "/usr/bin/valgrind")
set(CTEST_MEMORYCHECK_COMMAND "/usr/bin/valgrind")
set(CTEST_MEMORYCHECK_COMMAND_OPTIONS "-v --tool=memcheck --leak-check=full --track-fds=yes --num-callers=50 --show-reachable=yes --track-origins=yes --malloc-fill=0xff --free-fill=0xfe")

#-----------------------------------------------------------------------------
# Disable the warning when adding a subdir that has not CMakeLists.txt
#-----------------------------------------------------------------------------

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0014 OLD)
endif(COMMAND cmake_policy)

# Be sure to avoid troubles with library paths when using old policy
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Get rid of cmake RPATH warning on OSX
if(APPLE)
    set(CMAKE_MACOSX_RPATH OFF)
endif()

#-----------------------------------------------------------------------------
# Disable solution folders because not supported in VC Express
#-----------------------------------------------------------------------------

set_property(GLOBAL PROPERTY USE_FOLDERS Off)

#--------------------------------------------------------------
# === 1 === 
# Modify the project name if you want: 
#--------------------------------------------------------------

project(ChronoWheelDesigner)

#-----------------------------------------------------------------------------
# Location of additional CMake scripts
#-----------------------------------------------------------------------------

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")
include(GetGitRevisionDescription)
get_git_head_revision(GIT_REFSPEC VCS_REVISION)

if("${VCS_REVISION}" MATCHES ".*-NOTFOUND$")
    set(VCS_REVISION "")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#-----------------------------------------------------------------------------
# Enable CTest
#-----------------------------------------------------------------------------

enable_testing()
include(CTest)

#-----------------------------------------------------------------------------
# Use dependent options
#-----------------------------------------------------------------------------

include(CMakeDependentOption)

#-----------------------------------------------------------------------------
# Also, some variables that were used in previous makefile system (some
# of them could be set automatically because CMake should recognize automatically
# the compiler, generator, platform, etc....
# Set a default build type if none was specified
#-----------------------------------------------------------------------------

if(MSVC OR XCODE_VERSION)
    set(CMAKE_CONFIGURATION_TYPES Release CACHE STRING "Choose the type of build.")
    set_property(CACHE CMAKE_CONFIGURATION_TYPES PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    mark_as_advanced(FORCE CMAKE_BUILD_TYPE)
    mark_as_advanced(CLEAR CMAKE_CONFIGURATION_TYPES)
    set(CPACK_BUILD_TYPE ${CMAKE_CONFIGURATION_TYPES})
else()
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build.")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release MinSizeRel RelWithDebInfo)
    set(CPACK_BUILD_TYPE ${CMAKE_BUILD_TYPE})
endif()

if(XCODE_VERSION)
    set(CMAKE_XCODE_ATTRIBUTE_GCC_VERSION "com.apple.compilers.llvmgcc42")
endif()

#-----------------------------------------------------------------------------

set(CH_COMPILER " " CACHE STRING "Compiler Type" FORCE)

if(MINGW OR CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR CMAKE_COMPILER_IS_CLANGXX)
    if(CMAKE_SIZEOF_VOID_P MATCHES 4)
        set(CH_COMPILER "COMPILER_GCC" CACHE STRING "Compiler Type" FORCE)
    else()
        set(CH_COMPILER "COMPILER_GCC_X64" CACHE STRING "Compiler Type" FORCE)
    endif()
endif()

if(MSVC AND CMAKE_CL_64)
    set(CH_COMPILER "COMPILER_MSVC_X64" CACHE STRING "Compiler Type" FORCE)
elseif(MSVC)
    set(CH_COMPILER "COMPILER_MSVC" CACHE STRING "Compiler Type" FORCE)
endif()

#------------------------------------------------------------
# Install destinations for data and demo programs
#------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    set(CH_INSTALL_DATA "data")
    set(CH_INSTALL_DEMO "bin")
    set(CH_INSTALL_PYTHON "bin")
else()
    set(CH_INSTALL_DATA "share/chrono/data")
    set(CH_INSTALL_DEMO "share/chrono/bin")
    set(CH_INSTALL_PYTHON "share/chrono/python")
endif()

#--------------------------------------------------------------
# === 2 ===
# Find the Chrono package and any REQUIRED or OPTIONAL modules
# by invoking the find_package function in CONFIG mode:
#    find_package(Chrono
#	              COMPONENTS req_module1 req_module1 ...
#	              OPTIONAL opt_module1 opt_module2 ...
#                 CONFIG)
# The following Chrono modules can be requested (their names
# are case insensitive): Cascade, Cosimulation, FEA, Irrlicht,
# Matlab, Parallel, Postprocess, Python, Vehicle.
# 
# Note that you will have to set the variable Chrono_DIR to 
# specify the location of the ChronoConfig.cmake script, if
# it is not in its default install location.
# Chrono_DIR can be either a Chrono build tree or a Chrono install tree.
# 
# The following variables are set and can be used further down:
# CHRONO_FOUND
#   set to true if Chrono and all required components were found
# CHRONO_C_FLAGS
# CHRONO_CXX_FLAGS
#   C and C++ compilation flags
# CHRONO_INCLUDE_DIRS
#   additional paths for included headers
# CHRONO_LIBRARIES
#   list of required libraries (with full path)
# CHRONO_LINKER_FLAGS
#   additional linker flags
# CHRONO_DLLS
#   list of all DLL dependencies (with full path)
# CHRONO_DATA_DIR
#   path to the Chrono data make_directory
# 
# In addition, for each requested component [COMPONENT], the
# following variable is set to true (ON) or false (OFF):
# CHRONO_[COMPONENT]_FOUND
# 
# In this example, we only request the Irrlicht module (required)
#--------------------------------------------------------------

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib64")
find_package(Chrono
             COMPONENTS Irrlicht Fea Postprocess
             CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()


#--------------------------------------------------------------
# Add path to Chrono headers and to headers of all dependencies
# of the requested modules.
#--------------------------------------------------------------

include_directories(${CHRONO_INCLUDE_DIRS})

#--------------------------------------------------------------
# Tweaks to disable some warnings with MSVC
#--------------------------------------------------------------
if(MSVC)
    add_definitions("-D_CRT_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    add_definitions("-D_SCL_SECURE_NO_DEPRECATE")  # avoids deprecation warnings
    set(EXTRA_COMPILE_FLAGS "/wd4275")             # disable warnings triggered by Irrlicht
else()
    set(EXTRA_COMPILE_FLAGS "")
endif()

#--------------------------------------------------------------
# === 3 ===
# Add the executable from your project and specify all C++ 
# files in your project. 
#--------------------------------------------------------------

#add_executable(ChronoWheelDesigner ChronoWheelDesigner.cpp)

#--------------------------------------------------------------
# Set properties for your executable target
# 
# Note that here we define a macro CHRONO_DATA_DIR which will
# contain the path to the Chrono data directory, either in its
# source tree (if using a build version of Chrono), or in its
# install tree (if using an installed version of Chrono).
#--------------------------------------------------------------

#SOURCE_GROUP(""  FILES  "ChronoWheelDesigner.cpp")
#set_target_properties(ChronoWheelDesigner PROPERTIES 
#	    COMPILE_FLAGS "${CHRONO_CXX_FLAGS} ${EXTRA_COMPILE_FLAGS}"
#	    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\""
#	    LINK_FLAGS "${CHRONO_LINKER_FLAGS}")

#--------------------------------------------------------------
# Link to Chrono libraries and dependency libraries
#--------------------------------------------------------------

#target_link_libraries(ChronoWheelDesigner ${CHRONO_LIBRARIES})
#target_link_libraries(ChronoWheelDesigner ChronoEngine ChronoEngine_fea ChronoEngine_irrlicht)
#add_dependencies(ChronoWheelDesigner ChronoEngine ChronoEngine_fea ChronoEngine_irrlicht)
#install(TARGETS ChronoWheelDesigner DESTINATION ${CH_INSTALL_DEMO})

#--------------------------------------------------------------
# === 4 (OPTIONAL) ===
# 
# Optionally, add a custom command for copying all Chrono and
# dependency DLLs to the appropriate binary output folder.
# This function has effect only on Windows.
# 
# Note that you must first set EXECUTABLE_OUTPUT_PATH
# (this can simply be ${CMAKE_BINARY_DIR}, like in this example)
#--------------------------------------------------------------

#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib64)
add_DLL_copy_command("${CHRONO_DLLS}")

#------------------------------------------------------------
# Copy data directory to BUILD tree 
# Install data directory 
#------------------------------------------------------------

if(MSVC OR XCODE_VERSION)
    file(COPY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_BINARY_DIR}/bin/data/)
else()
    file(COPY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CMAKE_BINARY_DIR}/data/)
endif()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/ DESTINATION ${CH_INSTALL_DATA})

#------------------------------------------------------------
# Defer configuration of all Chrono libraries and programs
#------------------------------------------------------------

add_subdirectory(src)

#------------------------------------------------------------
# Use CPack
#------------------------------------------------------------

include(CPack)