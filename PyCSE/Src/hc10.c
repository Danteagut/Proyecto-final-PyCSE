#include "hc10.h"
#include "height_control.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>

/* Protocolo: "HEIGHT:<valor_cm>\n" (Node-RED -> Bluetooth -> HC-10 -> USART2). */

#define HC10_LINEA_MAX      32U

extern UART_HandleTypeDef huart2;

static uint8_t byte_rx = 0U;

static char     buffer_linea[HC10_LINEA_MAX];
static uint8_t  indice_linea = 0U;

static char             buffer_proceso[HC10_LINEA_MAX];
static volatile uint8_t linea_lista = 0U;

/* Arranca la recepción de 1 byte por interrupción sobre USART2. */
void HC10_Inicializar(void)
{
    indice_linea = 0U;
    linea_lista = 0U;
    HAL_UART_Receive_IT(&huart2, &byte_rx, 1U);
}

/* Callback HAL (nombre fijo): arma la línea byte a byte hasta '\n',
 * la copia a un buffer de proceso y vuelve a armar la recepción. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        if (byte_rx == '\n')
        {
            buffer_linea[indice_linea] = '\0';

            if (linea_lista == 0U)
            {
                memcpy(buffer_proceso, buffer_linea, indice_linea + 1U);
                linea_lista = 1U;
            }

            indice_linea = 0U;
        }
        else if (byte_rx != '\r')
        {
            if (indice_linea < (HC10_LINEA_MAX - 1U))
            {
                buffer_linea[indice_linea++] = (char)byte_rx;
            }
            else
            {
                /* Overflow: descartar línea inválida. */
                indice_linea = 0U;
            }
        }

        HAL_UART_Receive_IT(&huart2, &byte_rx, 1U);
    }
}

/* Valida el formato "HEIGHT:<numero>" y, si es correcto, actualiza el
 * setpoint del control de altura (el clamp 0-20 lo hace height_control). */
static void HC10_InterpretarLinea(const char *linea)
{
    const char *texto_valor;
    char       *fin_ptr;
    long        valor;

    if (strncmp(linea, "HEIGHT:", 7U) != 0)
    {
        return; /* comando desconocido: se ignora */
    }

    texto_valor = linea + 7U;

    if (*texto_valor == '\0')
    {
        return;
    }

    valor = strtol(texto_valor, &fin_ptr, 10);

    if ((fin_ptr == texto_valor) || (*fin_ptr != '\0'))
    {
        return; /* formato inválido: se ignora */
    }

    ControlAltura_FijarObjetivoCm((float)valor);
}

/* Si el callback de recepción marcó una línea lista, la procesa. */
void HC10_Procesar(void)
{
    if (linea_lista != 0U)
    {
        HC10_InterpretarLinea(buffer_proceso);
        linea_lista = 0U;
    }
}
