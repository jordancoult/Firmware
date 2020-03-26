px4_add_module(
    MODULE modules__morph_status
    MAIN morph_status
    STACK_MAIN 2500
    STACK_MAX 4000
    SRCS
        morph_status.cpp
    DEPENDS
        platforms__common

)