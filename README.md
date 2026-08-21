Motor Dron — Control de Altura por Realimentación Negativa

Trabajo práctico de posgrado en Sistemas Embebidos. Control de altura en lazo cerrado de un "drone conceptual" (motor DC + hélice sobre un banco de pruebas), implementado sobre un STM32F446RE (Nucleo-F446RE), con telemetría y consigna remota por Bluetooth Low Energy y un dashboard en Node-RED.

Qué hace

Un sensor ultrasónico HC-SR04 mide la altura real del conjunto. Un controlador PID corre en el STM32 a 10 Hz, compara esa altura contra un setpoint (objetivo) recibido de forma remota, y ajusta el empuje del motor (PWM sobre un puente H L298N) para sostenerlo en esa altura. El setpoint se manda con un simple comando de texto (HEIGHT:<cm>) desde un celular por BLE o desde la PC por el propio cable USB del ST-Link, y la telemetría se puede visualizar en tiempo real en un dashboard de Node-RED.

 objetivo (cm) ──►(+)──► error ──► PID ──► saturación ──► motor (PWM) ──► planta (altura real)
                   ▲                                                            │
                   └──────────────────── realimentación negativa (HC-SR04) ─────┘
