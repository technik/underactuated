################################################################################
# 2d pendulum model
################################################################################

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h" ../../libs/imgui/*.cpp ../../libs/implot/*.cpp)
GroupSources(src)
add_executable(pendulum ${SRC})
target_include_directories(pendulum PUBLIC
    ../../
    ../../libs/imgui
    ../../libs/implot
    src)
target_link_libraries(pendulum ${D3D12_LIBRARIES})