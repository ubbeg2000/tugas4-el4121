#include <stdio.h>
#include <stdlib.h>
#include "../fsm/rising_edge_fsm.h"

int main()
{
    int state = LOW_STATE, input, output = 0;
    FILE *f = fopen("rising_edge_simulation.csv", "w");

    fprintf(f, "input,output\n");
    for (int i = 0; i < 10; i++)
    {
        input = i < 5 ? 0 : 1;
        rising_edge_fsm(&state, input, &output);
        fprintf(f, "%d,%d\n", input, output);
    }
    for (int i = 0; i < 10; i++)
    {
        input = i < 5 ? 0 : 1;
        rising_edge_fsm(&state, input, &output);
        fprintf(f, "%d,%d\n", input, output);
    }

    fclose(f);
    return 0;
}