################################################################################
# 2d double pendulum (acrobot) model
################################################################################

file(GLOB_RECURSE SRC "src/*.cpp" "src/*.h" ../../libs/imgui/*.cpp ../../libs/implot/*.cpp)
GroupSources(src)
add_executable(acrobot ${SRC})
target_include_directories(acrobot PUBLIC
    ../../
    ../../libs/imgui
    ../../libs/implot
    src)
target_link_libraries(acrobot ${D3D12_LIBRARIES})