add_library(posting_policy INTERFACE)
target_include_directories(posting_policy INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_features(posting_policy INTERFACE cxx_std_17)

# --- tests ---
enable_testing()
add_executable(posting_policy_test test/posting_policy_test.cpp)
target_link_libraries(posting_policy_test PRIVATE posting_policy GTest::gtest_main)
add_test(NAME posting_policy_test COMMAND posting_policy_test)

add_library(result_source src/result_source.cpp)
target_include_directories(result_source PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(result_sink src/result_sink.cpp)
target_include_directories(result_sink PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
