Motor Dron — Control de Altura por Realimentación Negativa

Trabajo práctico de posgrado en Sistemas Embebidos. Control de altura en lazo cerrado de un "drone conceptual" (motor DC + hélice sobre un banco de pruebas), implementado sobre un STM32F446RE (Nucleo-F446RE), con telemetría y consigna remota por Bluetooth Low Energy y un dashboard en Node-RED.

Qué hace

Un sensor ultrasónico HC-SR04 mide la altura real del conjunto. Un controlador PID corre en el STM32 a 10 Hz, compara esa altura contra un setpoint (objetivo) recibido de forma remota, y ajusta el empuje del motor (PWM sobre un puente H L298N) para sostenerlo en esa altura. El setpoint se manda con un simple comando de texto (HEIGHT:<cm>) desde un celular por BLE o desde la PC por el propio cable USB del ST-Link, y la telemetría se puede visualizar en tiempo real en un dashboard de Node-RED.

 objetivo (cm) ──►(+)──► error ──► PID ──► saturación ──► motor (PWM) ──► planta (altura real)
                   ▲                                                            │
                   └──────────────────── realimentación negativa (HC-SR04) ─────┘
Hardware
Función	Pin STM32	Periférico
PWM motor (ENA del L298N)	PA5	TIM2_CH1
Dirección motor IN1	PB5	GPIO salida digital
Dirección motor IN2	PB10	GPIO salida digital
Trigger HC-SR04	PA9	GPIO salida digital
Echo HC-SR04	PA8	TIM1_CH1 (Input Capture)
Terminal de depuración	PA2 (TX) / PA3 (RX)	USART2, por el ST-Link (USB)
Módulo Bluetooth (BLE)	PA0 (TX) / PA1 (RX)	UART4

El módulo Bluetooth es un BT05 (chipset compatible HM-10, BLE puro — no Bluetooth clásico/SPP), en modo dato a 9600 baudios, expuesto sobre un breakout ZS-040. GND común obligatorio entre la alimentación del motor, el módulo y el STM32.

Arquitectura del firmware

El código está modularizado por responsabilidad, en vez de un único main.c:

Src/
├── main.c            → orquestador: inicializa todo y corre el superloop
├── motor.c            → driver L298N + PWM (TIM2_CH1/PA5)
├── distance.c          → driver HC-SR04 (TIM1 Input Capture) + filtro de promedio móvil
├── height_control.c    → controlador PID (no conoce hardware, solo calcula)
└── comandos.c          → recepción/envío por los dos enlaces (cable + Bluetooth)

Cada módulo se puede reemplazar o testear por separado: cambiar el sensor o el driver de motor no obliga a tocar el controlador, y viceversa. Se mantiene el estilo de configuración por registros de HAL/CMSIS (sin HAL_TIM_PWM_Init ni HAL_ADC) heredado de la etapa inicial del proyecto.

Control de altura (PID)

height_control.c implementa un PID con base de empuje, pensado para una planta de hélices (doble integrador), no para un motor con reductora:

empuje = EMPUJE_BASE + Kp·error + Ki·∫error + Kd·(−velocidad_vertical)
P corrige en proporción al error actual.
D deriva la altura medida (no el error, para evitar el "derivative kick" ante un setpoint nuevo) y amortigua: es el término que evita que el sistema oscile para siempre alrededor del objetivo.
I compensa el desajuste de EMPUJE_BASE (el punto de flotación real nunca cae exacto donde se lo estimó, y cambia con la carga de la batería), con anti-windup: solo se acumula si la salida no está saturada.
El motor nunca invierte el sentido de giro: para bajar se reduce el empuje por debajo del punto de flotación y el conjunto desciende por su propio peso, como una hélice real.

Las ganancias (EMPUJE_BASE_PCT, Kp, Ki, Kd) salen de una simulación con planta estimada — son un punto de partida seguro, no valores finales. El propio archivo trae el procedimiento de sintonía en banco (ajustar primero el punto de flotación, después Kd para eliminar oscilación, por último Ki para precisión).

Sensor de altura

HC-SR04 medido por Input Capture de hardware (no polling), con un filtro de promedio móvil de 4 muestras (ventana de 0,4 s a 10 Hz, deliberadamente corta para no atrasar el término derivativo). Si una medición falla (sin eco, fuera de rango), se conserva la última altura válida y no se contamina el filtro con un cero falso — el lazo de control nunca reacciona a una lectura inválida.

Comunicación: dos enlaces simultáneos

comandos.c escucha al mismo tiempo por dos UARTs independientes, ambas por interrupción:

CABLE — USART2 (115200 baud), el puerto serie virtual del propio ST-Link, sin hardware adicional.
BLUETOOTH — UART4 (9600 baud), hacia el módulo BT05.

Cualquiera de los dos acepta el comando de setpoint, y la telemetría sale por los dos a la vez (transmisión no bloqueante, para no frenar el lazo de control). Si el Bluetooth falla, se puede seguir operando por cable sin tocar el firmware.

Protocolo de entrada (setpoint, 3–20 cm con clamp):

HEIGHT:<valor_cm>\n

Protocolo de telemetría (cada 500 ms):

H:<altura_cm>,SP:<setpoint_cm>,V:<empuje_%>,VZ:<vel_vertical_cm/s>,S:<sensor_ok>,L:<CABLE|BT>\n
Dashboard Node-RED

Node-RED/control-altura-dashboard.json da una interfaz web para operar y visualizar el lazo sin recompilar nada:

Slider (3–20 cm) y botones de escalón rápido (5 cm → 15 cm) para ensayos de respuesta al escalón.
Gráfico de altura medida vs. setpoint en el tiempo.
Gauge de empuje aplicado (%) y gráfico de velocidad vertical (para sintonizar Kd).
Configuración serie independiente para CABLE y BLUETOOTH — si el módulo es BLE puro (como el BT05), Windows no expone un puerto COM clásico para él, así que ese nodo conviene deshabilitarlo y operar el setpoint desde una app BLE en el celular (ej. LightBlue, escribiendo sobre la característica GATT del módulo) mientras el dashboard muestra la telemetría por el cable.
Levantar el dashboard
bash
npm install
npx node-red

Importar Node-RED/control-altura-dashboard.json y ajustar el puerto COM del nodo "CABLE (ST-Link)" al que le haya asignado Windows al Nucleo.

Estructura del repositorio
├── Src/, Inc/              → firmware principal (STM32CubeIDE)
├── Drivers/                → HAL + CMSIS
├── Startup/, *.ld           → arranque y linker
├── Motor_dron_PyCSE.ioc     → configuración CubeMX
├── Node-RED/                → dashboard de control y telemetría
├── package.json              → dependencias npm del dashboard Node-RED
└── variante_4_motores/       → experimento aislado: extensión a 4 motores (2 canales L298N,
                                 TIM2 + TIM3), sin tocar el firmware principal
Compilar y flashear

Abrir el proyecto en STM32CubeIDE (importar Motor_dron_PyCSE.ioc / la carpeta como proyecto existente), compilar y flashear por el ST-Link integrado del Nucleo.

Estado / roadmap
Las ganancias del PID son un punto de partida simulado: falta la sintonía final en banco con el equipo real (procedimiento documentado en height_control.c).
El dashboard Node-RED ya está preparado para recibir el enlace Bluetooth como puerto serie clásico; con un módulo BLE puro como el actual, esa vía requeriría un nodo BLE dedicado (ej. node-red-contrib-noble-bluetooth) en vez de serial-port.
