add_executable(catnav2 catnav2.cpp)

target_link_libraries(catnav2 PRIVATE project_warnings project_options
                                      Nav2remote Obstacle)
