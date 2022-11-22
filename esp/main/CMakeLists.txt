idf_component_register(SRCS "esp.c" "../../fsm/debounce_fsm.c" "../../fsm/rising_edge_fsm.c" "../../fsm/mode_fsm.c" "../../fsm/autorepeat_fsm.c" "../../controller/controller.c"
                    INCLUDE_DIRS "." "../../fsm" "../../controller")
