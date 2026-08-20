#ifndef HEIGHT_CONTROL_H
#define HEIGHT_CONTROL_H

#include <stdint.h>

/* Período del lazo de control. Vive acá y no en main.c porque el término
 * derivativo lo necesita para calcular la velocidad: si los dos valores se
 * separaran, la derivada quedaría mal escalada sin dar ningún error. */
#define CONTROL_PERIODO_MS   100U

/* Deja el lazo en reposo: motor detenido, esperando el primer setpoint.
 * No mueve el motor hasta recibir un HEIGHT válido por HC-10. */
void ControlAltura_Inicializar(void);

/* Fija el nuevo setpoint (cm), aplicando clamp estricto [3, 20]. */
void ControlAltura_FijarObjetivoCm(float objetivo_cm);

/* Ejecuta una iteración del lazo PD. Llamar cada CONTROL_PERIODO_MS. */
void ControlAltura_Actualizar(float altura_actual_cm);

/* Getters para telemetría / depuración. */
float       ControlAltura_ObtenerObjetivoCm(void);
float       ControlAltura_ObtenerErrorCm(void);
float       ControlAltura_ObtenerVelocidadVerticalCmS(void);
float       ControlAltura_ObtenerIntegralPct(void);
uint8_t     ControlAltura_ObtenerEmpujePct(void);
const char *ControlAltura_ObtenerSentidoTexto(void);

#endif /* HEIGHT_CONTROL_H */
