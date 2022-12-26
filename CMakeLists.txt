project(HypnoSquid)

set(CMAKE_CXX_STANDARD 23)

add_subdirectory(HypnoSquid.Core)
add_subdirectory(TestPlugin)

add_executable(Test main.cc)
target_link_libraries(Test HypnoSquid.Core)
add_dependencies(Test TestPlugin)

configure_file(hypnosquid.json hypnosquid.json COPYONLY)