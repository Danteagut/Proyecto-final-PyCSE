#include "height_control.h"
#include "motor.h"

#define ALTURA_MIN_CM      0.0f
#define ALTURA_MAX_CM      20.0f
#define HISTERESIS_CM      1.0f   /* banda ±1.0 cm: sensor sin filtro/promediado */

static float objetivo_cm = 0.0f;

/* Arranca con setpoint 0 cm hasta que llegue el primer HEIGHT: por HC-10. */
void ControlAltura_Inicializar(void)
{
    objetivo_cm = 0.0f;
}

/* Guarda el setpoint recibido, limitado estrictamente al rango 0-20 cm. */
void ControlAltura_FijarObjetivoCm(float objetivo)
{
    if (objetivo < ALTURA_MIN_CM)
    {
        objetivo = ALTURA_MIN_CM;
    }
    else if (objetivo > ALTURA_MAX_CM)
    {
        objetivo = ALTURA_MAX_CM;
    }

    objetivo_cm = objetivo;
}

/* error = objetivo - altura_actual, con banda de histéresis para evitar
 * oscilaciones del motor cerca del setpoint. */
void ControlAltura_Actualizar(float altura_actual_cm)
{
    if (altura_actual_cm < (objetivo_cm - HISTERESIS_CM))
    {
        Motor_FijarEstado(MOTOR_SUBIR);
    }
    else if (altura_actual_cm > (objetivo_cm + HISTERESIS_CM))
    {
        Motor_FijarEstado(MOTOR_BAJAR);
    }
    else
    {
        Motor_FijarEstado(MOTOR_DETENER);
    }
}
