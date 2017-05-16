rock_library(range_sensor_micro_epsilon
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(range_sensor_micro_epsilon_bin Main.cpp
    DEPS range_sensor_micro_epsilon)

