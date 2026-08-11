#ifndef HEIGHT_CONTROL_H
#define HEIGHT_CONTROL_H

/* Inicializa el setpoint de altura en 0 cm. */
void ControlAltura_Inicializar(void);

/* Fija el nuevo setpoint (cm), aplicando clamp estricto [0, 20]. */
void ControlAltura_FijarObjetivoCm(float objetivo_cm);

/* Compara altura actual vs setpoint (con histéresis) y comanda el motor. */
void ControlAltura_Actualizar(float altura_actual_cm);

#endif /* HEIGHT_CONTROL_H */
