#ifndef HC10_H
#define HC10_H

/* Arranca la recepción UART por interrupción (no bloqueante). */
void HC10_Inicializar(void);

/* Si hay una línea completa recibida, la valida/parsea y actualiza
 * el setpoint de height_control. Llamar en cada vuelta del loop. */
void HC10_Procesar(void);

#endif /* HC10_H */
