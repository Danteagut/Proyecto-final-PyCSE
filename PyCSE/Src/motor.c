#include "motor.h"
#include "main.h"

/* Driver L298N + PWM (TIM2_CH1/PA5) existente, modularizado.
 * Velocidad fija (no viene de Node-RED ni del potenciómetro). */
#define MOTOR_VELOCIDAD_FIJA_PORCENTAJE   60UL
#define PWM_PERIODO_CUENTAS               1000UL   /* ARR + 1 */

#define L298N_PIN_IN1                     5U       /* PB5  - D4 */
#define L298N_PIN_IN2                     10U      /* PB10 - D6 */

/* Confirmado con el usuario: FORWARD (IN1=1,IN2=0) hace que la altura
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

/* Fija el duty cycle del PWM como porcentaje (0-100) de la velocidad. */
static void Motor_FijarVelocidadPorcentaje(uint32_t porcentaje)
{
    if (porcentaje > 100UL)
    {
        porcentaje = 100UL;
    }

    TIM2->CCR1 = (porcentaje * PWM_PERIODO_CUENTAS) / 100UL;
}

/* Configura PB5/PB10 como salidas digitales para IN1/IN2 del L298N. */
static void Configurar_PinesDireccion_L298N(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    GPIOB->MODER &= ~((3UL << (L298N_PIN_IN1 * 2U)) | (3UL << (L298N_PIN_IN2 * 2U)));
    GPIOB->MODER |=  ((1UL << (L298N_PIN_IN1 * 2U)) | (1UL << (L298N_PIN_IN2 * 2U)));
    GPIOB->OTYPER &= ~((1UL << L298N_PIN_IN1) | (1UL << L298N_PIN_IN2));
    GPIOB->OSPEEDR &= ~((3UL << (L298N_PIN_IN1 * 2U)) | (3UL << (L298N_PIN_IN2 * 2U)));
    GPIOB->PUPDR &= ~((3UL << (L298N_PIN_IN1 * 2U)) | (3UL << (L298N_PIN_IN2 * 2U)));
}

/* IN1=1, IN2=0: gira en el sentido que hace SUBIR la altura. */
static void Motor_GirarAdelante(void)
{
    GPIOB->BSRR = (1UL << L298N_PIN_IN2) | (1UL << (L298N_PIN_IN1 + 16U));
}

/* IN1=0, IN2=1: gira en el sentido que hace BAJAR la altura. */
static void Motor_GirarAtras(void)
{
    GPIOB->BSRR = (1UL << (L298N_PIN_IN2 + 16U)) | (1UL << L298N_PIN_IN1);
}

/* IN1=0, IN2=0: frena el motor (rueda libre / freno según el driver). */
static void Motor_Detener(void)
{
    GPIOB->BSRR = (1UL << (L298N_PIN_IN1 + 16U)) | (1UL << (L298N_PIN_IN2 + 16U));
}

/* Inicializa PWM, dirección y deja el motor detenido a velocidad fija. */
void Motor_Inicializar(void)
{
    Configurar_PWM_TIM2_PA5();
    Configurar_PinesDireccion_L298N();
    Motor_Detener();
    Motor_FijarVelocidadPorcentaje(MOTOR_VELOCIDAD_FIJA_PORCENTAJE);
}

/* Traduce el comando de height_control (subir/bajar/detener) a hardware. */
void Motor_FijarEstado(MotorComando_t comando)
{
    switch (comando)
    {
        case MOTOR_SUBIR:
            Motor_GirarAdelante();
            break;
        case MOTOR_BAJAR:
            Motor_GirarAtras();
            break;
        case MOTOR_DETENER:
        default:
            Motor_Detener();
            break;
    }
}
