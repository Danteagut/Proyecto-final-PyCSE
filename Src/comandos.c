#include "comandos.h"
#include "height_control.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>

/* Protocolo de entrada:  "HEIGHT:<valor_cm>\n"
 *
 * Se acepta por los dos enlaces indistintamente. El clamp 3-20 lo hace
 * height_control, no este módulo, aca lo unico que hago es decirle al sistema
 * de donde voy a sacar los datos, lo implemente cuando me empezo a fallar el bluetooth
 *  */

#define LINEA_MAX        32U
#define CANT_ENLACES      2U

#define IDX_CABLE         0U   /* USART2 - ST-Link por USB */
#define IDX_BLUETOOTH     1U   /* UART4  - HC-05 / HC-10   */

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart4;

/* Estado de recepción de un enlace. Cada puerto arma su línea por separado,
 * así los dos pueden estar recibiendo al mismo tiempo sin mezclarse. */
typedef struct
{
    UART_HandleTypeDef *huart;
    uint8_t             byte_rx;         /* destino de HAL_UART_Receive_IT */
    char                buffer[LINEA_MAX];
    uint8_t             indice;
    char                linea_lista[LINEA_MAX];
    volatile uint8_t    hay_linea;
    char                buffer_tx[96];   /* propio: la transmisión es por IT */
} Enlace_t;

static Enlace_t enlaces[CANT_ENLACES];

static EnlaceOrigen_t ultimo_origen = ENLACE_NINGUNO;

/* Devuelve el enlace al que pertenece un handle, o NULL si no es ninguno. */
static Enlace_t *Buscar_Enlace(const UART_HandleTypeDef *huart)
{
    uint8_t i;

    for (i = 0U; i < CANT_ENLACES; i++)
    {
        if (enlaces[i].huart == huart)
        {
            return &enlaces[i];
        }
    }

    return NULL;
}

/* Arranca la recepción por interrupción en los dos puertos. */
void Comandos_Inicializar(void)
{
    uint8_t i;

    memset(enlaces, 0, sizeof(enlaces));

    enlaces[IDX_CABLE].huart     = &huart2;
    enlaces[IDX_BLUETOOTH].huart = &huart4;

    ultimo_origen = ENLACE_NINGUNO;

    for (i = 0U; i < CANT_ENLACES; i++)
    {
        (void)HAL_UART_Receive_IT(enlaces[i].huart, &enlaces[i].byte_rx, 1U);
    }
}

/* Callback HAL (nombre fijo): arma la línea byte a byte hasta '\n' en el
 * enlace que corresponda, y vuelve a armar la recepción de ese puerto. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Enlace_t *e = Buscar_Enlace(huart);

    if (e == NULL)
    {
        return;
    }

    if (e->byte_rx == (uint8_t)'\n')
    {
        e->buffer[e->indice] = '\0';

        /* Si la línea anterior todavía no se procesó, se descarta esta en
         * lugar de pisarla a medias. */
        if (e->hay_linea == 0U)
        {
            memcpy(e->linea_lista, e->buffer, (size_t)e->indice + 1U);
            e->hay_linea = 1U;
        }

        e->indice = 0U;
    }
    else if (e->byte_rx != (uint8_t)'\r')
    {
        if (e->indice < (LINEA_MAX - 1U))
        {
            e->buffer[e->indice] = (char)e->byte_rx;
            e->indice++;
        }
        else
        {
            e->indice = 0U;   /* desborde: línea inválida, se descarta */
        }
    }
    else
    {
        /* '\r' se ignora */
    }

    (void)HAL_UART_Receive_IT(e->huart, &e->byte_rx, 1U);
}

/* Valida el formato "HEIGHT:<numero>" y actualiza el setpoint.
 * Devuelve 1 si el comando era válido. */
static uint8_t Interpretar_Linea(const char *linea)
{
    const char *texto_valor;
    char       *fin_ptr;
    long        valor;

    if (strncmp(linea, "HEIGHT:", 7U) != 0)
    {
        return 0U;   /* comando desconocido: se ignora */
    }

    texto_valor = linea + 7U;

    if (*texto_valor == '\0')
    {
        return 0U;
    }

    valor = strtol(texto_valor, &fin_ptr, 10);

    if ((fin_ptr == texto_valor) || (*fin_ptr != '\0'))
    {
        return 0U;   /* formato inválido: se ignora */
    }

    ControlAltura_FijarObjetivoCm((float)valor);

    return 1U;
}

/* Procesa lo que haya llegado por cualquiera de los dos enlaces. */
void Comandos_Procesar(void)
{
    uint8_t i;

    for (i = 0U; i < CANT_ENLACES; i++)
    {
        if (enlaces[i].hay_linea != 0U)
        {
            if (Interpretar_Linea(enlaces[i].linea_lista) != 0U)
            {
                ultimo_origen = (i == IDX_CABLE) ? ENLACE_CABLE : ENLACE_BLUETOOTH;
            }

            enlaces[i].hay_linea = 0U;
        }
    }
}

/* Envía por los dos puertos SIN BLOQUEAR.
 *
 * Se usa transmisión por interrupción a propósito: a 9600 baudios una línea
 * de telemetría tarda unos 30 ms, y con HAL_UART_Transmit bloqueante eso
 * frenaba el lazo de control, que corre cada 100 ms. Un retraso ahí falsea
 * el cálculo de la velocidad del término derivativo.
 *
 * Si el puerto todavía está transmitiendo lo anterior, se saltea ese envío:
 * perder un reporte no tiene consecuencia, frenar el lazo sí. */
void Comandos_EnviarTelemetria(const char *texto)
{
    uint8_t i;
    size_t  largo = strlen(texto);

    for (i = 0U; i < CANT_ENLACES; i++)
    {
        if (largo >= sizeof(enlaces[i].buffer_tx))
        {
            continue;
        }

        if (enlaces[i].huart->gState != HAL_UART_STATE_READY)
        {
            continue;   /* ocupado: se saltea este reporte */
        }

        memcpy(enlaces[i].buffer_tx, texto, largo + 1U);

        (void)HAL_UART_Transmit_IT(enlaces[i].huart,
                                   (uint8_t *)enlaces[i].buffer_tx,
                                   (uint16_t)largo);
    }
}

/* Por dónde entró el último comando válido. */
EnlaceOrigen_t Comandos_ObtenerUltimoOrigen(void)
{
    return ultimo_origen;
}

/* Texto del enlace activo, para los mensajes. */
const char *Comandos_ObtenerUltimoOrigenTexto(void)
{
    switch (ultimo_origen)
    {
        case ENLACE_CABLE:     return "CABLE";
        case ENLACE_BLUETOOTH: return "BT";
        default:               return "-";
    }
}
