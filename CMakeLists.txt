project(HypnoSquid)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_FLAGS "-gdwarf-4")

add_subdirectory(HypnoSquid.Core)
add_subdirectory(TestPlugin)

add_executable(Test main.cc)
target_link_libraries(Test HypnoSquid.Core)
add_dependencies(Test TestPlugin)

add_executable(SandExample sand_example.cc)
target_link_libraries(SandExample HypnoSquid.Core)
add_dependencies(SandExample TestPlugin)

configure_file(hypnosquid.json hypnosquid.json COPYONLY)