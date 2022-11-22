autorepeat-fsm-simulation:
	gcc -o autorepeat_fsm_simulation ./fsm/autorepeat_fsm_simulation.c ./fsm/autorepeat_fsm.c
	
controller-simulation:
	gcc -o controller-simulation ./controller/controller_simulation.c ./motor/motor.c ./controller/controller.c

debounce-fsm-simulation:
	gcc -o debounce-fsm-simulation ./fsm/debounce_fsm_simulation.c ./fsm/debounce_fsm.c

rising-edge-fsm-simulation:
	gcc -o rising-edge-fsm-simulation ./fsm/rising_edge_fsm_simulation.c ./fsm/rising_edge_fsm.c
