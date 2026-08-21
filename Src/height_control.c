#include "height_control.h"
#include "motor.h"
#include <math.h>

/* ---------------------------------------------------------------------------
 * Lazo PID con base de empuje, para elevación por HÉLICES.
 *
 *   salida = EMPUJE_BASE + Kp·error + Ki·∫error + Kd·(−velocidad)
 *
 * Orden de importancia: primero D (estabilidad), después I (precisión).
 * Un P+I sin D sobre esta planta es inestable; el D tiene que estar puesto.
 *
 *  * ------------------------------------------------------------------------- */

/* El HC-SR04 no mide de forma confiable por debajo de ~2 cm. */
#define ALTURA_MIN_CM          3.0f
#define ALTURA_MAX_CM          20.0f

/* Duty al que el conjunto queda al borde de despegar, MEDIDO en el drone
 * con pruebas, si cambia el peso o algo deberia volver a medirlo */
#define EMPUJE_BASE_PCT        88.0f

/* Ganancia proporcional: % de empuje por cada cm de error.
 * hice algunas pruebas y por ahora esto es lo mejor, podria hacer una simualcion */
#define KP_PCT_POR_CM           0.40f

/* Ganancia integral: % de empuje por cada cm·s de error acumulado. */
#define KI_PCT_POR_CM_S         0.2f

/* Ganancia derivativa: % de empuje por cada cm/s de velocidad vertical. */
#define KD_PCT_POR_CM_S         0.9f

/* Zona muerta sobre el término proporcional. lo fui probando
 * hasta que dejo de temblar el empuje, le doy 0,5cm porque es
 * el posible error */
#define ZONA_MUERTA_CM          0.5f

/* Saturación de la salida, alrededor de la base.
 * El mínimo no es cero: conviene que las hélices sigan girando para no
 * pagar el tiempo de arranque cada vez que hay que descender. */
#define SALIDA_MIN_PCT         40.0f
#define SALIDA_MAX_PCT         95.0f

#define PERIODO_S              ((float)CONTROL_PERIODO_MS / 1000.0f)

static float   objetivo_cm         = ALTURA_MIN_CM;
static float   error_cm            = 0.0f;
static float   velocidad_cm_s      = 0.0f;
static float   altura_anterior_cm  = 0.0f;
static uint8_t hay_altura_anterior = 0U;
static uint8_t empuje_pct          = 0U;

/* Acumulador del término integral, en % de empuje. */
static float   integral_pct        = 0.0f;

/* Mientras valga 0 el lazo no mueve el motor. Evita que al encender, con
 * un setpoint que el operador todavía no eligió, las hélices arranquen
 * solas. Se habilita con el primer HEIGHT válido. */
static uint8_t objetivo_recibido   = 0U;

/* Deja el lazo en reposo: motores frenados, esperando el primer setpoint. */
void ControlAltura_Inicializar(void)
{
    objetivo_cm         = ALTURA_MIN_CM;
    error_cm            = 0.0f;
    velocidad_cm_s      = 0.0f;
    altura_anterior_cm  = 0.0f;
    hay_altura_anterior = 0U;
    empuje_pct          = 0U;
    integral_pct        = 0.0f;
    objetivo_recibido   = 0U;

    Motor_Aplicar(MOTOR_DETENER, 0U);
}

/* Guarda el setpoint recibido por HC-10, limitado estrictamente a 3-20 cm. */
void ControlAltura_FijarObjetivoCm(float objetivo)
{
    if (objetivo < ALTURA_MIN_CM)
    {
        objetivo = ALTURA_MIN_CM;
    }
    else if (objetivo > ALTURA_MAX_CM)
    {
        objetivo = ALTURA_MAX_CM;
    }

    objetivo_cm       = objetivo;
    objetivo_recibido = 1U;   /* habilita el lazo */
}

/* Una iteración del lazo PD. */
void ControlAltura_Actualizar(float altura_actual_cm)
{
    float termino_p;
    float termino_d;
    float salida_pct;
    float salida_sin_saturar;

    /* Sin setpoint elegido todavía, las hélices no giran. */
    if (objetivo_recibido == 0U)
    {
        error_cm            = 0.0f;
        velocidad_cm_s      = 0.0f;
        empuje_pct          = 0U;
        integral_pct        = 0.0f;
        altura_anterior_cm  = altura_actual_cm;
        hay_altura_anterior = 1U;

        Motor_Aplicar(MOTOR_DETENER, 0U);
        return;
    }

    /* --- Velocidad vertical -------------------------------------------
     * Se deriva la ALTURA MEDIDA, no el error. Si se derivara el error,
     * cada vez que Node-RED manda un setpoint nuevo el salto brusco
     * produciría un pico enorme en la derivada
     *
     * Además la altura ya viene filtrada por el promedio móvil: derivar
     * la señal cruda amplificaría el ruido del sensor. */
    if (hay_altura_anterior == 0U)
    {
        velocidad_cm_s = 0.0f;   /* primera iteración: no hay con qué comparar */
        hay_altura_anterior = 1U;
    }
    else
    {
        velocidad_cm_s = (altura_actual_cm - altura_anterior_cm) / PERIODO_S;
    }

    altura_anterior_cm = altura_actual_cm;

    /* --- Realimentación negativa --------------------------------------- */
    error_cm = objetivo_cm - altura_actual_cm;

    /* --- Término proporcional -----------------------------------------
     * Sin valor absoluto ni lógica de signo: si el error es negativo
     * (está por encima del objetivo) el término resta empuje y el
     * conjunto baja solo. La gravedad hace el trabajo. */
    if (fabsf(error_cm) <= ZONA_MUERTA_CM)
    {
        termino_p = 0.0f;
    }
    else
    {
        termino_p = KP_PCT_POR_CM * error_cm;
    }

    /* --- Término derivativo -------------------------------------------
     * Signo negativo: si el conjunto sube, descuenta empuje para frenar
     * antes de alcanzar el objetivo. */
    termino_d = -KD_PCT_POR_CM_S * velocidad_cm_s;

    salida_sin_saturar = EMPUJE_BASE_PCT + termino_p + integral_pct + termino_d;

    /* --- Saturación ---------------------------------------------------- */
    salida_pct = salida_sin_saturar;

    if (salida_pct > SALIDA_MAX_PCT)
    {
        salida_pct = SALIDA_MAX_PCT;
    }
    else if (salida_pct < SALIDA_MIN_PCT)
    {
        salida_pct = SALIDA_MIN_PCT;
    }

    /* --- Término integral, con ANTI-WINDUP -----------------------------
     * Sólo se acumula si la salida NO está saturada. sino se haria infinito*/
    if ((salida_pct == salida_sin_saturar) && (KI_PCT_POR_CM_S > 0.0f))
    {
        integral_pct += KI_PCT_POR_CM_S * error_cm * PERIODO_S;
    }

    empuje_pct = (uint8_t)salida_pct;

    /* Las hélices giran en un solo sentido: el sentido NUNCA se invierte.
     * Para descender se reduce el empuje y baja por su propio peso. */
    Motor_Aplicar(MOTOR_SUBIR, empuje_pct);
}

/* Setpoint vigente, en cm. */
float ControlAltura_ObtenerObjetivoCm(void)
{
    return objetivo_cm;
}

/* Error de la última iteración (objetivo - altura), en cm. */
float ControlAltura_ObtenerErrorCm(void)
{
    return error_cm;
}

/* Velocidad vertical estimada, en cm/s. Positiva = subiendo. */
float ControlAltura_ObtenerVelocidadVerticalCmS(void)
{
    return velocidad_cm_s;
}

/* Empuje que el lazo le pidió a los motores, en %. */
uint8_t ControlAltura_ObtenerEmpujePct(void)
{
    return empuje_pct;
}

/* Qué está haciendo el lazo respecto de la base de empuje. */
const char *ControlAltura_ObtenerSentidoTexto(void)
{
    if (empuje_pct == 0U)
    {
        return "PARADO";
    }

    if ((float)empuje_pct > (EMPUJE_BASE_PCT + 1.0f))
    {
        return "SUBE";
    }

    if ((float)empuje_pct < (EMPUJE_BASE_PCT - 1.0f))
    {
        return "BAJA";
    }

    return "MANTIENE";
}

/* Corrección acumulada por el integral, en % de empuje. Sirve para
 * sintonizar: si al estabilizarse vale +3, la base real está en 63 % y
 * conviene subir EMPUJE_BASE_PCT a ese valor. */
float ControlAltura_ObtenerIntegralPct(void)
{
    return integral_pct;
}
