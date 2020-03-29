# automatically enable catch2 to generate ctest targets

include(/usr/local/lib/cmake/Catch2/Catch.cmake)

add_library(catch_main STATIC catch_main.cpp)
target_link_libraries(catch_main PUBLIC)

add_executable(utest-nmpc_algebra utest-nmpc_algebra.cpp)
target_link_libraries(
  utest-nmpc_algebra PRIVATE project_warnings project_options catch_main
                             Obstacle)

add_executable(utest-Obstacle utest-Obstacle.cpp)
target_link_libraries(utest-Obstacle PRIVATE project_warnings project_options
                                             catch_main Obstacle)

add_executable(utest-XY utest-XY.cpp)
target_link_libraries(utest-XY PRIVATE project_warnings project_options
                                       catch_main)

# automatically discover tests that are defined in catch based test files you
# can modify the unittests. TEST_PREFIX to whatever you want, or use different
# for different binaries
catch_discover_tests(
  utest-nmpc_algebra
  TEST_PREFIX
  "utests."
  EXTRA_ARGS
  -s
  --reporter=xml
  --out=tests.xml)
catch_discover_tests(
  utest-Obstacle
  TEST_PREFIX
  "utests."
  EXTRA_ARGS
  -s
  --reporter=xml
  --out=tests.xml)
catch_discover_tests(
  utest-XY
  TEST_PREFIX
  "utests."
  EXTRA_ARGS
  -s
  --reporter=xml
  --out=tests.xml)
