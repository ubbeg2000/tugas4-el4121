#include "mode_fsm.h"

void mode_fsm(int *state, int input)
{
    switch (*state)
    {
    case OPERATING_MODE:
    {
        if (input == 1)
        {
            *state = SET_P_MODE;
        }
        break;
    }
    case SET_P_MODE:
    {
        if (input == 1)
        {
            *state = SET_I_MODE;
        }
        break;
    }
    case SET_I_MODE:
    {
        if (input == 1)
        {
            *state = SET_D_MODE;
        }
        break;
    }
    case SET_D_MODE:
    {
        if (input == 1)
        {
            *state = OPERATING_MODE;
        }
        break;
    }
    default:
        break;
    }
}