// coils.h
#ifndef COILS_H
#define COILS_H

#include <Arduino.h>

// 4 bobin için ID
enum CoilId {
  COIL1 = 0,
  COIL2,
  COIL3,
  COIL4
};

// L298N + bobin pinlerini hazırlar (setup'te 1 kez çağır)
void coilsInit();

// Tek bir bobini sür (u: -255 .. +255)
void coilSetPWM(CoilId id, float u);

// Tüm bobinleri kapat
void coilsOff();

#endif // COILS_H
