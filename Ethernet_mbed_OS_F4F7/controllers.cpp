#include "controllers.h"




// the classic PID
void PID_Controller(
        float *error,
        float *kp_switch,
        float *kp,
        float *p_portion,
        float *ki_switch,
        float *ki,
        float *integral,
        float *i_portion,
        float *kd_switch,
        float *kd,
        float *lastError, 
        float *intervalLength,
        float *d_portion,
        float *setPoint
    )

{
    // prohibit divison by zero
    if (*intervalLength == 0)
    {
        return;
    }
    
    // integrate the error
    *integral += (*error * *intervalLength);
    
    // calculate proportional portion
    *p_portion = *kp_switch * *kp * *error;
    
    // calculate integral portion
    *i_portion = *ki_switch * *ki * *integral;
    
    // calculate derivative portion
    *d_portion = *kd_switch * *kd * ((*error - *lastError)/(*intervalLength));
    
    // calculate set point
    *setPoint = *p_portion + *i_portion + *d_portion;
    
    // save the new error for the next calculation
    *lastError = *error;
}
