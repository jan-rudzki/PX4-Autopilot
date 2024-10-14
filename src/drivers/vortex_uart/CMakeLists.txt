px4_add_module(
    MODULE drivers__vortex_uart
    MAIN uart_interface
    STACK_MAIN 2048
    SRCS
        uart_interface.cpp
    DEPENDS
)