#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* Sentido de giro pedido por el control de altura. */
typedef enum
{
    MOTOR_DETENER = 0,
    MOTOR_SUBIR,
    MOTOR_BAJAR
} MotorComando_t;

/* Inicializa PWM (TIM2_CH1/PA5) y pines de dirección (L298N), motor detenido. */
void Motor_Inicializar(void);

/* Aplica sentido + velocidad (0-100 %) calculada por el control proporcional.
 * La velocidad ya NO es una constante interna: la fija el lazo de control. */
void Motor_Aplicar(MotorComando_t sentido, uint8_t velocidad_pct);

#endif /* MOTOR_H */
