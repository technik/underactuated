################################################################################
# Experiments with liquid neural networks and simplified vehicle driving
################################################################################

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h")
GroupSources(src)
add_executable(liquid1 ${SRC})
target_include_directories(liquid1 PUBLIC
    ../../
    ../../libs/imgui
    ../../libs/implot
    src)
target_link_libraries(liquid1 core ${D3D12_LIBRARIES})