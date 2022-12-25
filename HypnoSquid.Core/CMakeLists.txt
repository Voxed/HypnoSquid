add_library(HypnoSquid.Core src/Engine.cc include/Engine.hh include/Query.hh include/ComponentReference.hh include/ComponentState.hh include/TypeIterator.hh include/common.hh include/Filter.hh include/Commands.hh)
target_include_directories(HypnoSquid.Core PUBLIC include)

find_package(nlohmann_json 3.11.2 REQUIRED)

target_link_libraries(HypnoSquid.Core PRIVATE nlohmann_json::nlohmann_json)