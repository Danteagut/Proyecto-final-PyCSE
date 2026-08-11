#include "distance.h"
#include "main.h"
#include <stdio.h>

/* Driver HC-SR04 existente, modularizado. TIM1 (Input Capture) se sigue
 * inicializando en main.c (MX_TIM1_Init); acá sólo se usa el handle. */

#define HCSR04_TRIG_PORT             GPIOA
#define HCSR04_TRIG_PIN              GPIO_PIN_9   /* D8 / PA9 */
#define HCSR04_TIMEOUT_MS            60U
#define TIM1_MAX_COUNT                65535UL

extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;

static volatile uint8_t  estado_captura = 0U;
static volatile uint8_t  medicion_lista = 0U;
static volatile uint32_t flanco_subida = 0U;
static volatile uint32_t flanco_bajada = 0U;
static volatile uint32_t pulso_us = 0U;

static float altura_ultima_cm = 0.0f;

/* Espera activa en microsegundos usando TIM1 como base de tiempo. */
static void Retardo_Microsegundos(uint32_t us)
{
    uint32_t inicio = __HAL_TIM_GET_COUNTER(&htim1);

    while ((__HAL_TIM_GET_COUNTER(&htim1) - inicio) < us)
    {
        /* Espera activa */
    }
}

/* Genera el pulso TRIG de 10us y deja el TIM1 listo para capturar ECHO. */
static void Disparar_Sensor(void)
{
    estado_captura = 0U;
    medicion_lista = 0U;
    flanco_subida = 0U;
    flanco_bajada = 0U;
    pulso_us = 0U;

    __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);

    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
    Retardo_Microsegundos(2U);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    Retardo_Microsegundos(10U);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);
}

/* Dispara una medición y espera (con timeout) el ancho de pulso ECHO;
 * devuelve 1 y la distancia en décimas de cm si fue válida, 0 si no. */
static uint8_t Leer_Distancia_IC(uint32_t *p_pulso_us, uint32_t *p_distancia_x10_cm)
{
    uint32_t inicio_tick;

    Disparar_Sensor();
    inicio_tick = HAL_GetTick();

    while (medicion_lista == 0U)
    {
        if ((HAL_GetTick() - inicio_tick) > HCSR04_TIMEOUT_MS)
        {
            estado_captura = 0U;
            __HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
            return 0U;
        }
    }

    *p_pulso_us = pulso_us;
    /* distancia_x10_cm = pulso_us * 1715 / 10000, redondeado */
    *p_distancia_x10_cm = (((*p_pulso_us) * 1715UL) + 5000UL) / 10000UL;

    return 1U;
}

/* Callback HAL de Input Capture (nombre fijo por la HAL): registra flanco
 * de subida y de bajada de ECHO para calcular el ancho de pulso. */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint32_t valor_capturado;

    if ((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        valor_capturado = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        if (estado_captura == 0U)
        {
            flanco_subida = valor_capturado;
            estado_captura = 1U;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        }
        else if (estado_captura == 1U)
        {
            flanco_bajada = valor_capturado;

            if (flanco_bajada >= flanco_subida)
            {
                pulso_us = flanco_bajada - flanco_subida;
            }
            else
            {
                pulso_us = (TIM1_MAX_COUNT - flanco_subida) + flanco_bajada + 1UL;
            }

            medicion_lista = 1U;
            estado_captura = 2U;
            __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        }
    }
}

/* Envía por USART2 el pulso y la distancia medidos (o el error), para debug. */
static void Enviar_Distancia_USART2(uint8_t valido, uint32_t pulso_us_local, uint32_t distancia_x10_cm)
{
    char buffer_tx[128];
    int len;

    if (valido != 0U)
    {
        len = snprintf(buffer_tx, sizeof(buffer_tx),
                       "Echo=%5lu us | Distancia=%lu.%lu cm\r\n",
                       (unsigned long)pulso_us_local,
                       (unsigned long)(distancia_x10_cm / 10UL),
                       (unsigned long)(distancia_x10_cm % 10UL));
    }
    else
    {
        len = snprintf(buffer_tx, sizeof(buffer_tx), "Error: sin eco o fuera de rango\r\n");
    }

    if (len > 0)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer_tx, (uint16_t)len, HAL_MAX_DELAY);
    }
}

/* Mide altura actual y actualiza el valor cacheado; se llama periódicamente. */
void Distancia_Actualizar(void)
{
    uint32_t pulso_us_local = 0U;
    uint32_t distancia_x10_cm = 0U;
    uint8_t valido = Leer_Distancia_IC(&pulso_us_local, &distancia_x10_cm);

    if (valido != 0U)
    {
        altura_ultima_cm = (float)distancia_x10_cm / 10.0f;
    }

    Enviar_Distancia_USART2(valido, pulso_us_local, distancia_x10_cm);
}

/* Devuelve la última altura válida (cm) para el control de altura. */
float Distancia_ObtenerCm(void)
{
    return altura_ultima_cm;
}
