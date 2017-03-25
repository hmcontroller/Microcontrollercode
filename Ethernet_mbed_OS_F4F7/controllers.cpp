#include "controllers.h"

void PID_Controller(PidParameters *params)
{
    // prohibit divison by zero
    if (*(params->intervalLength) == 0)
    {
        return;
    }

    // integrate the error
    *(params->integral) += (*(params->error) * *(params->intervalLength));

    // calculate proportional portion
    *(params->pPortion) = *(params->kpSwitch) * *(params->kpValue) * *(params->error);

    // calculate integral portion
    *(params->iPortion) = *(params->kiSwitch) * *(params->kiValue) * *(params->integral);

    // calculate derivative portion
    *(params->dPortion) = *(params->kdSwitch) * *(params->kdValue) * ((*(params->error) - *(params->lastError))/(*(params->intervalLength)));

    // calculate set point
    *(params->setPoint) = *(params->pPortion) + *(params->iPortion) + *(params->dPortion);

    // save the new error for the next calculation
    *(params->lastError) = *(params->error);
}
