rock_library(range_sensor_micro_epsilon
    SOURCES range_sensor.cpp packet_types.hpp
    HEADERS packet_types.hpp range_sensor.hpp
    DEPS_PKGCONFIG base-lib iodrivers_base base-logging)

rock_executable(range_sensor_micro_epsilon_bin Main.cpp
    DEPS range_sensor_micro_epsilon)

