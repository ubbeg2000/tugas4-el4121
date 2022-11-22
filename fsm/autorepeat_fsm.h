#define AUTOREPEAT_OFF_STATE 10
#define AUTOREPEAT_WAIT_STATE 11
#define AUTOREPEAT_STATE 12

#define AUTOREPEAT_WAIT_DURATION 100
#define AUTOREPEAT_PERIOD 1

void autorepeat_fsm(int *state, int *cnt, int input, int *output);
