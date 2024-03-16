################################################################################
# Experiments with liquid neural networks and simplified vehicle driving
################################################################################

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h" ../../libs/imgui/*.cpp ../../libs/implot/*.cpp)
GroupSources(src)
add_executable(liquid1 ${SRC})
target_include_directories(liquid1 PUBLIC
    ../../
    ../../libs/imgui
    ../../libs/implot
    src)
target_link_libraries(liquid1 ${D3D12_LIBRARIES})