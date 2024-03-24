################################################################################
# Experiments with liquid neural networks and simplified vehicle driving
################################################################################
project(liquid1)

file(GLOB_RECURSE LIQUID1_SRC "src/*.cpp" "src/*.h" "../../ml/*.h")
GroupSources(src liquid1)
GroupSources(../../ml ml)
add_executable(liquid1 ${LIQUID1_SRC})
target_include_directories(liquid1 PUBLIC
    ../../
    ../../ml
    src)
target_link_libraries(liquid1 core ${D3D12_LIBRARIES})