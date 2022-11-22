#include <stdio.h>
#include "../fsm/debounce_fsm.h"

int bounce[10] = {0, 1, 0, 1, 1, 0, 1, 0, 1, 0};
int zeros[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int ones[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

int main()
{
    FILE *f = fopen("debounce_fsm_simulation.csv", "w");
    int state = WAITING_STATE, cnt = 0, output = 0;

    fprintf(f, "input,output\n");

    for (int i = 0; i < 10; i++)
    {
        debounce_fsm(&state, &cnt, bounce[i], &output);
        fprintf(f, "%d,%d\n", bounce[i], output);
    }

    for (int i = 0; i < 10; i++)
    {
        debounce_fsm(&state, &cnt, ones[i], &output);
        fprintf(f, "%d,%d\n", ones[i], output);
    }

    for (int i = 0; i < 10; i++)
    {
        debounce_fsm(&state, &cnt, bounce[i], &output);
        fprintf(f, "%d,%d\n", bounce[i], output);
    }

    for (int i = 0; i < 10; i++)
    {
        debounce_fsm(&state, &cnt, zeros[i], &output);
        fprintf(f, "%d,%d\n", zeros[i], output);
    }

    for (int i = 0; i < 10; i++)
    {
        debounce_fsm(&state, &cnt, bounce[i], &output);
        fprintf(f, "%d,%d\n", bounce[i], output);
    }

    for (int i = 0; i < 10; i++)
    {
        debounce_fsm(&state, &cnt, ones[i], &output);
        fprintf(f, "%d,%d\n", ones[i], output);
    }

    for (int i = 0; i < 10; i++)
    {
        debounce_fsm(&state, &cnt, bounce[i], &output);
        fprintf(f, "%d,%d\n", bounce[i], output);
    }

    for (int i = 0; i < 10; i++)
    {
        debounce_fsm(&state, &cnt, zeros[i], &output);
        fprintf(f, "%d,%d\n", zeros[i], output);
    }

    fclose(f);
    return 0;
}