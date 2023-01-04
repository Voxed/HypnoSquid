add_library(HypnoSquid.Core SHARED src/Engine.cc include/HypnoSquid.Core/Engine.hh include/HypnoSquid.Core/world_extension/Query.hh include/HypnoSquid.Core/world_extension/ComponentReference.hh include/HypnoSquid.Core/world_extension/ComponentState.hh include/HypnoSquid.Core/common.hh include/HypnoSquid.Core/world_extension/Filter.hh include/HypnoSquid.Core/world_extension/Commands.hh include/HypnoSquid.Core/world_extension/WorldExtension.hh include/HypnoSquid.Core/InvocationID.hh include/HypnoSquid.Core/Extension.hh include/HypnoSquid.Core/PluginName.hh)
target_include_directories(HypnoSquid.Core PUBLIC include)
target_include_directories(HypnoSquid.Core PRIVATE include/HypnoSquid.Core)

find_package(nlohmann_json 3.11.2 REQUIRED)

target_link_libraries(HypnoSquid.Core PRIVATE nlohmann_json::nlohmann_json)