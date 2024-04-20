# Copyright (c) 2024 BiologIC Technologies Ltd
zephyr_syscall_header(${CMAKE_CURRENT_SOURCE_DIR}/../../include/zephyr/smartpump.h)
zephyr_sources_ifdef(CONFIG_SMARTPUMP smartpump.c)
zephyr_library_sources(smartpump.c)


