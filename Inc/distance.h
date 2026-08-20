#ifndef DISTANCE_H
#define DISTANCE_H

#include <stdint.h>

/* Dispara una medición HC-SR04, la filtra y actualiza el valor cacheado.
 * Ya no imprime nada: el reporte lo hace main.c a su propio ritmo. */
void Distancia_Actualizar(void);

/* Última altura filtrada, en cm (entrada del lazo de control). */
float Distancia_ObtenerCm(void);

/* 1 si la última medición fue válida, 0 si hubo timeout / sin eco. */
uint8_t Distancia_UltimaFueValida(void);

#endif /* DISTANCE_H */
