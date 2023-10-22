################################################################################
# 2d pendulum model
################################################################################

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h" ../../../libs/imgui/*.cpp ../../../libs/implot/*.cpp)
GroupSources(src)
add_executable(segway ${SRC})
target_include_directories(segway PUBLIC
    ../../../
    ../../../libs/imgui
    ../../../libs/implot
    src)
target_link_libraries(segway ${D3D12_LIBRARIES})