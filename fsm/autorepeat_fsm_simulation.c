#include <stdio.h>
#include <stdlib.h>
#include "autorepeat_fsm.h"

int main()
{
    int state = LOW_STATE, cnt = 0, input = 0, output = 0;
    FILE *f = fopen("autorepeat_fsm_simulation.csv", "w");

    fprintf(f, "input,output\n");

    for (int i = 0; i < 500; i++)
    {
        input = i > 50 && i < 300 ? 1 : 0;
        autorepeat_fsm(&state, &cnt, input, &output);
        fprintf(f, "%d,%d\n", input, output);
    }

    fclose(f);

    return 0;
}