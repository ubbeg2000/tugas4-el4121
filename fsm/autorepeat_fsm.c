#include "autorepeat_fsm.h"

void autorepeat_fsm(int *state, int *cnt, int input, int *output)
{
    switch (*state)
    {
    case AUTOREPEAT_OFF_STATE:
    {
        if (input == 1)
        {
            *cnt = 0;
            *output = 1;
            *state = AUTOREPEAT_WAIT_STATE;
        }
        else
        {
            *cnt = 0;
            *output = 0;
            *state = AUTOREPEAT_OFF_STATE;
        }
        break;
    }
    case AUTOREPEAT_WAIT_STATE:
    {
        if (input == 1)
        {
            if (*cnt >= AUTOREPEAT_WAIT_DURATION)
            {
                *cnt = 0;
                *output = 0;
                *state = AUTOREPEAT_STATE;
            }
            else
            {
                *cnt += 1;
                *output = 0;
                *state = AUTOREPEAT_WAIT_STATE;
            }
        }
        else
        {
            *cnt = 0;
            *output = 0;
            *state = AUTOREPEAT_OFF_STATE;
        }
        break;
    }
    case AUTOREPEAT_STATE:
    {
        if (input == 1)
        {
            if (*cnt >= AUTOREPEAT_PERIOD)
            {
                *cnt = 0;
                *output = 1;
                *state = AUTOREPEAT_STATE;
            }
            else
            {
                *cnt += 1;
                *output = 0;
                *state = AUTOREPEAT_STATE;
            }
        }
        else
        {
            *cnt = 0;
            *output = 0;
            *state = AUTOREPEAT_OFF_STATE;
        }
        break;
    }

    default:
        break;
    }
}