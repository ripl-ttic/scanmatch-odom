add_executable(scanmatch-odom scan_matcher.cpp)

pods_use_pkg_config_packages(scanmatch-odom lcm bot2-core scanmatch bot2-param-client lcmtypes_scanmatch bot2-frames)

pods_install_executables(scanmatch-odom)
