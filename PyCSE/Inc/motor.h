#ifndef MOTOR_H
#define MOTOR_H

/* Comando de sentido para el motor; la velocidad es fija e interna. */
typedef enum
{
    MOTOR_DETENER = 0,
    MOTOR_SUBIR,
    MOTOR_BAJAR
} MotorComando_t;

/* Inicializa PWM (TIM2_CH1/PA5), pines de dirección (L298N) y velocidad fija. */
void Motor_Inicializar(void);

/* Ejecuta el sentido pedido por height_control (subir/bajar/detener). */
void Motor_FijarEstado(MotorComando_t comando);

#endif /* MOTOR_H */
