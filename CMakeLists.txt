################################################################################
# 2d pendulum model
################################################################################
cmake_minimum_required (VERSION 3.20)
project(underactuated)

# Global options
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(CMAKE_CXX_STANDARD 20)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/modules")

# Clean Windows headers
add_definitions(-DWIN32_LEAN_AND_MEAN -DWIN32_EXTRA_LEAN -DNOMINMAX)

find_package(D3D12 REQUIRED)

macro(GroupSources curdir)
    file(GLOB children RELATIVE ${PROJECT_SOURCE_DIR}/${curdir}
${PROJECT_SOURCE_DIR}/${curdir}/*)
    foreach(child ${children})
        if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/${curdir}/${child})
            GroupSources(${curdir}/${child})
        else()
            string(REPLACE "/" "\\" groupname ${curdir})
            source_group(${groupname} FILES ${PROJECT_SOURCE_DIR}/${curdir}/${child})
          endif()
   endforeach()
endmacro()

###################################################################################################
macro(compile_HLSL_flags _SOURCE SOURCE_LIST OUTPUT_LIST _PROFILE)
    get_filename_component(FILE_NAME ${_SOURCE} NAME)
    set(OUTPUT ${FILE_NAME}.cso)
    set(ASM ${FILE_NAME}.asm)

    LIST(APPEND ${SOURCE_LIST} ${_SOURCE})
    LIST(APPEND ${OUTPUT_LIST} ${OUTPUT})

    set(_COMMAND dxc -E main -Fc ${ASM} -Zi -Fd ${FILE_NAME}.pdb -Fo ${OUTPUT} -I "../src/shaders/" -T ${_PROFILE} ${_SOURCE})
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/bin/${OUTPUT}
        COMMAND echo ${_COMMAND}
        COMMAND ${_COMMAND}
        MAIN_DEPENDENCY ${_SOURCE}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin
        )
endmacro()

add_subdirectory(models/acrobot)
add_subdirectory(models/pendulum)