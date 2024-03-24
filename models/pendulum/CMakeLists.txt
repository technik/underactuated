################################################################################
# 2d pendulum model
################################################################################
project(pendulum)

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h")
GroupSources(src pendulum)
add_executable(pendulum ${SRC})
target_include_directories(pendulum PUBLIC
    ../../
    ../../libs/imgui
    ../../libs/implot
    src)
target_link_libraries(pendulum core ${D3D12_LIBRARIES})