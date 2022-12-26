add_library(HypnoSquid.Core src/Engine.cc include/HypnoSquid.Core/Engine.hh include/HypnoSquid.Core/Query.hh include/HypnoSquid.Core/ComponentReference.hh include/HypnoSquid.Core/ComponentState.hh include/HypnoSquid.Core/TypeIterator.hh include/HypnoSquid.Core/common.hh include/HypnoSquid.Core/Filter.hh include/HypnoSquid.Core/Commands.hh)
target_include_directories(HypnoSquid.Core PUBLIC include)
target_include_directories(HypnoSquid.Core PRIVATE include/HypnoSquid.Core)

find_package(nlohmann_json 3.11.2 REQUIRED)

target_link_libraries(HypnoSquid.Core PRIVATE nlohmann_json::nlohmann_json)