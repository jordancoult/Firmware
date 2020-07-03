px4_add_module(
    MODULE modules__dynageo
    MAIN dynageo
    STACK_MAIN 2500
    STACK_MAX 4000
    SRCS
        dynageo.cpp
)