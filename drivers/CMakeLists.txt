
add_subdirectory(pump)

add_subdirectory_ifdef(CONFIG_SENSOR sensor)

zephyr_include_directories(include)
