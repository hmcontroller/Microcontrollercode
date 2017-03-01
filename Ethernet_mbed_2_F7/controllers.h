#ifndef CONTROLLERS_H
#define CONTROLLERS_H

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
        );


#endif
