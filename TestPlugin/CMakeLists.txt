add_library(TestPlugin SHARED src/TestPlugin.cc)
target_include_directories(TestPlugin PUBLIC include)

target_link_libraries(TestPlugin HypnoSquid.Core)