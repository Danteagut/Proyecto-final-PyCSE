#ifndef DISTANCE_H
#define DISTANCE_H

/* Dispara una medición HC-SR04 y actualiza el valor cacheado de altura. */
void Distancia_Actualizar(void);

/* Devuelve la última altura válida medida, en cm. */
float Distancia_ObtenerCm(void);

#endif /* DISTANCE_H */
