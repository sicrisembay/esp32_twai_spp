set(srcs    "main.c"
            "wifi_network.c"
            "can.c"
            "tcp_server.c")

set(include_dirs ".")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${include_dirs})