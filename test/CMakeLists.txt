add_library (CATCH catch.cpp)

macro (create_test target)
  add_test (NAME ${target} COMMAND $<TARGET_FILE:${target}>)
endmacro (create_test)

set(testname utest-nmpc_algebra)
    add_executable (${testname} ${testname}.cpp)
    target_link_libraries (${testname} CATCH Obstacle)
    create_test(${testname})

set(testname utest-Obstacle)
    add_executable (${testname} ${testname}.cpp)
    target_link_libraries (${testname} CATCH Obstacle)
    create_test(${testname})

set(testname utest-XY)
    add_executable (${testname} ${testname}.cpp)
    target_link_libraries (${testname} CATCH Obstacle)
    create_test(${testname})
