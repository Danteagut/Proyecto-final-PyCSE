#include "distance.h"
#include "main.h"

/* Driver HC-SR04 existente, modularizado. TIM1 (Input Capture) se sigue
 * inicializando en main.c (MX_TIM1_Init); acá sólo se usa el handle. */

#define HCSR04_TRIG_PORT             GPIOA
#define HCSR04_TRIG_PIN              GPIO_PIN_9   /* D8 / PA9 */
#define HCSR04_TIMEOUT_MS            60U
#define TIM1_MAX_COUNT                65535UL

/* Filtro de PROMEDIO MÓVIL sobre la altura medida (mismo método que se
 * usaba sobre el ADC del potenciómetro). El lazo proporcional amplifica el
 * ruido del sensor por Kp, así que hay que suavizar la entrada.
 *
 * Con 4 muestras y el lazo a 10 Hz la ventana es de 0,4 s. Se usa una
 * ventana corta a propósito: el término derivativo del control deriva esta
 * señal, y el retardo del filtro se le suma directamente. Con 8 muestras el
 * retardo era suficiente para desestabilizar el lazo. */
#define FILTRO_MUESTRAS              4U

extern TIM_HandleTypeDef htim1;

static volatile uint8_t  estado_captura = 0U;
static volatile uint8_t  medicion_lista = 0U;
static volatile uint32_t flanco_subida = 0U;
static volatile uint32_t flanco_bajada = 0U;
static volatile uint32_t pulso_us = 0U;

static float   altura_filtrada_cm = 0.0f;
static uint8_t ultima_valida = 0U;

/* Estado del promedio móvil. Se trabaja en décimas de cm (enteros) para
 * no arrastrar error de redondeo al acumular, igual que el filtro del ADC. */
static uint32_t buffer_filtro[FILTRO_MUESTRAS] = {0U};
static uint32_t indice_filtro   = 0U;  /* dónde se guarda la próxima muestra */
static uint32_t suma_filtro     = 0U;  /* suma de las muestras del buffer */
static uint32_t cantidad_filtro = 0U;  /* muestras válidas cargadas */

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

/* Promedio móvil: mantiene una ventana con las últimas FILTRO_MUESTRAS
 * mediciones y devuelve su promedio, en décimas de cm.
 *
 * Mientras el buffer no se llenó, promedia sobre las muestras que hay, así
 * la primera lectura ya devuelve un valor útil y no arranca desde cero.
 * Una vez lleno, resta la muestra más vieja y suma la nueva: el buffer es
 * circular, no hace falta recorrerlo entero en cada llamada. */
static uint32_t Filtrar_PromedioMovil(uint32_t nueva_muestra_x10)
{
    if (cantidad_filtro < FILTRO_MUESTRAS)
    {
        buffer_filtro[indice_filtro] = nueva_muestra_x10;
        suma_filtro += nueva_muestra_x10;
        cantidad_filtro++;
    }
    else
    {
        suma_filtro -= buffer_filtro[indice_filtro];
        buffer_filtro[indice_filtro] = nueva_muestra_x10;
        suma_filtro += nueva_muestra_x10;
    }

    indice_filtro++;

    if (indice_filtro >= FILTRO_MUESTRAS)
    {
        indice_filtro = 0U;
    }

    return (suma_filtro / cantidad_filtro);
}

/* Mide la altura, la filtra y actualiza el valor cacheado.
 * Si la medición falla se conserva la última válida y NO se carga al
 * filtro, para que el lazo de control no reaccione a un cero falso. */
void Distancia_Actualizar(void)
{
    uint32_t pulso_us_local = 0U;
    uint32_t distancia_x10_cm = 0U;
    uint32_t promedio_x10;

    ultima_valida = Leer_Distancia_IC(&pulso_us_local, &distancia_x10_cm);

    if (ultima_valida == 0U)
    {
        return;
    }

    promedio_x10 = Filtrar_PromedioMovil(distancia_x10_cm);

    altura_filtrada_cm = (float)promedio_x10 / 10.0f;
}

/* Altura filtrada (cm): es la realimentación que entra al lazo de control. */
float Distancia_ObtenerCm(void)
{
    return altura_filtrada_cm;
}

/* Indica si la última medición fue válida, para reportarlo por depuración. */
uint8_t Distancia_UltimaFueValida(void)
{
    return ultima_valida;
}
