#ifndef COMANDOS_H
#define COMANDOS_H

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Recepción de consignas desde Node-RED por DOS enlaces simultáneos:
 *
 *   CABLE      USART2 (PA2/PA3) -> Virtual COM Port del ST-Link, por el USB
 *   BLUETOOTH  UART4  (PA0/PA1) -> módulo HC-05 / HC-10
 *
 * Los dos están siempre escuchando y aceptan el mismo comando. No hay modo
 * que elegir ni nada que recompilar: Node-RED se conecta al puerto COM que
 * corresponda y ese enlace queda operativo. Si el módulo Bluetooth falla,
 * se trabaja por cable sin tocar el firmware.
 *
 * La telemetría sale por los dos puertos a la vez, así que se ve igual
 * desde cualquiera de los dos.
 * ------------------------------------------------------------------------- */

typedef enum
{
    ENLACE_NINGUNO = 0,
    ENLACE_CABLE,
    ENLACE_BLUETOOTH
} EnlaceOrigen_t;

/* Arranca la recepción por interrupción en los dos puertos. */
void Comandos_Inicializar(void);

/* Procesa las líneas completas que hayan llegado por cualquiera de los dos
 * enlaces. Llamar en cada vuelta del bucle principal. */
void Comandos_Procesar(void);

/* Envía una cadena por los dos puertos, sin bloquear. */
void Comandos_EnviarTelemetria(const char *texto);

/* Por dónde entró el último comando válido. Sirve para mostrar en la
 * telemetría con qué enlace se está trabajando. */
EnlaceOrigen_t Comandos_ObtenerUltimoOrigen(void);

/* Texto del enlace activo, para los mensajes. */
const char *Comandos_ObtenerUltimoOrigenTexto(void);

#endif /* COMANDOS_H */
