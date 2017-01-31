#add_definitions(
#    -Wno-ignored-qualifiers
#    )
set(sm_sources
    scan_matcher.cpp 
    )

add_executable(hr-scan-match ${sm_sources})

pods_use_pkg_config_packages(hr-scan-match lcm bot2-core lcmtypes_hr-lcmtypes scanmatch bot2-param-client lcmtypes_scanmatch bot2-frames)

pods_install_executables(hr-scan-match)
