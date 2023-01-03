add_library(HypnoSquid.Core SHARED src/Engine.cc include/HypnoSquid.Core/Engine.hh include/HypnoSquid.Core/components/Query.hh include/HypnoSquid.Core/components/ComponentReference.hh include/HypnoSquid.Core/components/ComponentState.hh include/HypnoSquid.Core/common.hh include/HypnoSquid.Core/components/Filter.hh include/HypnoSquid.Core/components/Commands.hh include/HypnoSquid.Core/components/ComponentExtension.hh include/HypnoSquid.Core/InvocationID.hh)
target_include_directories(HypnoSquid.Core PUBLIC include)
target_include_directories(HypnoSquid.Core PRIVATE include/HypnoSquid.Core)

find_package(nlohmann_json 3.11.2 REQUIRED)

target_link_libraries(HypnoSquid.Core PRIVATE nlohmann_json::nlohmann_json)