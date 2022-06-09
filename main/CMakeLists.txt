set(srcs    "twai_spp_main.c"
            "bt_bridge.c"
            "frame_parser.c")

set(include_dirs ".")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ${include_dirs})