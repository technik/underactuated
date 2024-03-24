################################################################################
# 2d pendulum model
################################################################################

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h")
GroupSources(src segway)
add_executable(segway ${SRC})
target_include_directories(segway PUBLIC
    ../../../
    ../../../libs/imgui
    ../../../libs/implot
    src)
target_link_libraries(segway core ${D3D12_LIBRARIES})