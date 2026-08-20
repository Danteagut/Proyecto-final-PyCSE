#include "motor.h"
#include "main.h"

/* Driver L298N con sus 2 canales de potencia sincronizados para mover
 * 4 motores DC (2 en paralelo por canal): canal A = ENA/PA5 (TIM2_CH1) +
 * IN1/IN2, canal B = ENB/PA6 (TIM3_CH1) + IN3/IN4. Ambos canales reciben
 * siempre el mismo sentido y duty; ya no se puentean ENA-ENB por hardware.
 * La velocidad la determina el lazo proporcional de height_control,
 * no Node-RED ni el potenciómetro. */
#define PWM_PERIODO_CUENTAS               1000UL   /* ARR + 1 */

#define L298N_PIN_IN1                     5U       /* PB5  - D4  (canal A) */
#define L298N_PIN_IN2                     10U      /* PB10 - D6  (canal A) */
#define L298N_PIN_IN3                     4U       /* PB4  - D5  (canal B) */
#define L298N_PIN_IN4                     6U       /* PB6  - D10 (canal B) */

/* FORWARD (IN1=1,IN2=0 / IN3=1,IN4=0) hace que la altura
 * medida por el sensor AUMENTE => FORWARD = MOTOR_SUBIR. */

/* Configura TIM2_CH1 sobre PA5 como salida PWM de 1 kHz (señal ENA del L298N). */
static void Configurar_PWM_TIM2_PA5(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    GPIOA->MODER &= ~(3UL << (5U * 2U));
    GPIOA->MODER |=  (2UL << (5U * 2U));
    GPIOA->OTYPER &= ~(1UL << 5U);
    GPIOA->OSPEEDR &= ~(3UL << (5U * 2U));
    GPIOA->OSPEEDR |=  (2UL << (5U * 2U));
    GPIOA->PUPDR &= ~(3UL << (5U * 2U));

    GPIOA->AFR[0] &= ~(0xFUL << (5U * 4U));
    GPIOA->AFR[0] |=  (1UL   << (5U * 4U));

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;

    TIM2->CR1 &= ~TIM_CR1_CEN;

    TIM2->PSC = 83U;
    TIM2->ARR = 999U;
    TIM2->CNT = 0U;
    TIM2->CCR1 = 0U;

    TIM2->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM2->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM2->CCMR1 |=  (6UL << 4U);
    TIM2->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM2->CCER &= ~TIM_CCER_CC1P;
    TIM2->CCER |=  TIM_CCER_CC1E;

    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->CR1 &= ~TIM_CR1_CMS;
    TIM2->CR1 &= ~TIM_CR1_DIR;

    TIM2->EGR |= TIM_EGR_UG;
    TIM2->CR1 |= TIM_CR1_CEN;
}

/* Configura TIM3_CH1 sobre PA6 como salida PWM de 1 kHz (señal ENB del L298N,
 * canal B). Misma configuración que el canal A pero con AF2 (TIM3). */
static void Configurar_PWM_TIM3_PA6(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void)RCC->AHB1ENR;

    GPIOA->MODER &= ~(3UL << (6U * 2U));
    GPIOA->MODER |=  (2UL << (6U * 2U));
    GPIOA->OTYPER &= ~(1UL << 6U);
    GPIOA->OSPEEDR &= ~(3UL << (6U * 2U));
    GPIOA->OSPEEDR |=  (2UL << (6U * 2U));
    GPIOA->PUPDR &= ~(3UL << (6U * 2U));

    GPIOA->AFR[0] &= ~(0xFUL << (6U * 4U));
    GPIOA->AFR[0] |=  (2UL   << (6U * 4U));

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    (void)RCC->APB1ENR;

    TIM3->CR1 &= ~TIM_CR1_CEN;

    TIM3->PSC = 83U;
    TIM3->ARR = 999U;
    TIM3->CNT = 0U;
    TIM3->CCR1 = 0U;

    TIM3->CCMR1 &= ~TIM_CCMR1_CC1S;
    TIM3->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM3->CCMR1 |=  (6UL << 4U);
    TIM3->CCMR1 |= TIM_CCMR1_OC1PE;

    TIM3->CCER &= ~TIM_CCER_CC1P;
    TIM3->CCER |=  TIM_CCER_CC1E;

    TIM3->CR1 |= TIM_CR1_ARPE;
    TIM3->CR1 &= ~TIM_CR1_CMS;
    TIM3->CR1 &= ~TIM_CR1_DIR;

    TIM3->EGR |= TIM_EGR_UG;
    TIM3->CR1 |= TIM_CR1_CEN;
}

/* Traduce un porcentaje de velocidad (0-100) al registro de comparación
 * de ambos PWM (ENA y ENB), que define el duty cycle de los 4 motores. */
static void Motor_FijarVelocidadPorcentaje(uint32_t porcentaje)
{
    uint32_t cuentas;

    if (porcentaje > 100UL)
    {
        porcentaje = 100UL;
    }

    cuentas = (porcentaje * PWM_PERIODO_CUENTAS) / 100UL;
    TIM2->CCR1 = cuentas;
    TIM3->CCR1 = cuentas;
}

/* Configura PB4/PB5/PB6/PB10 como salidas digitales para IN1..IN4 del L298N. */
static void Configurar_PinesDireccion_L298N(void)
{
    uint32_t mascara_2b = (3UL << (L298N_PIN_IN1 * 2U)) | (3UL << (L298N_PIN_IN2 * 2U))
                         | (3UL << (L298N_PIN_IN3 * 2U)) | (3UL << (L298N_PIN_IN4 * 2U));
    uint32_t valor_2b   = (1UL << (L298N_PIN_IN1 * 2U)) | (1UL << (L298N_PIN_IN2 * 2U))
                         | (1UL << (L298N_PIN_IN3 * 2U)) | (1UL << (L298N_PIN_IN4 * 2U));
    uint32_t mascara_1b = (1UL << L298N_PIN_IN1) | (1UL << L298N_PIN_IN2)
                         | (1UL << L298N_PIN_IN3) | (1UL << L298N_PIN_IN4);

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    GPIOB->MODER &= ~mascara_2b;
    GPIOB->MODER |=  valor_2b;
    GPIOB->OTYPER &= ~mascara_1b;
    GPIOB->OSPEEDR &= ~mascara_2b;
    GPIOB->PUPDR &= ~mascara_2b;
}

/* IN1=1,IN2=0 / IN3=1,IN4=0: los 2 canales giran en el sentido que hace
 * SUBIR la altura. */
static void Motor_GirarAdelante(void)
{
    GPIOB->BSRR = (1UL << L298N_PIN_IN1) | (1UL << L298N_PIN_IN3)
                | (1UL << (L298N_PIN_IN2 + 16U)) | (1UL << (L298N_PIN_IN4 + 16U));
}

/* IN1=0,IN2=1 / IN3=0,IN4=1: los 2 canales giran en el sentido que hace
 * BAJAR la altura. */
static void Motor_GirarAtras(void)
{
    GPIOB->BSRR = (1UL << L298N_PIN_IN2) | (1UL << L298N_PIN_IN4)
                | (1UL << (L298N_PIN_IN1 + 16U)) | (1UL << (L298N_PIN_IN3 + 16U));
}

/* Los 4 IN en 0: frena los 4 motores (rueda libre / freno según el driver). */
static void Motor_Detener(void)
{
    GPIOB->BSRR = (1UL << (L298N_PIN_IN1 + 16U)) | (1UL << (L298N_PIN_IN2 + 16U))
                | (1UL << (L298N_PIN_IN3 + 16U)) | (1UL << (L298N_PIN_IN4 + 16U));
}

/* Inicializa los 2 PWM (ENA/ENB) y las 4 direcciones, y deja los 4 motores
 * detenidos con duty en cero. */
void Motor_Inicializar(void)
{
    Configurar_PWM_TIM2_PA5();
    Configurar_PWM_TIM3_PA6();
    Configurar_PinesDireccion_L298N();
    Motor_Detener();
    Motor_FijarVelocidadPorcentaje(0UL);
}

/* Aplica el resultado del lazo proporcional: primero el sentido en IN1..IN4
 * y después el duty del PWM sobre ENA/ENB (los 4 motores en sincronismo).
 * Al detener fuerza duty 0. */
void Motor_Aplicar(MotorComando_t sentido, uint8_t velocidad_pct)
{
    switch (sentido)
    {
        case MOTOR_SUBIR:
            Motor_GirarAdelante();
            Motor_FijarVelocidadPorcentaje((uint32_t)velocidad_pct);
            break;

        case MOTOR_BAJAR:
            Motor_GirarAtras();
            Motor_FijarVelocidadPorcentaje((uint32_t)velocidad_pct);
            break;

        case MOTOR_DETENER:
        default:
            Motor_Detener();
            Motor_FijarVelocidadPorcentaje(0UL);
            break;
    }
}
