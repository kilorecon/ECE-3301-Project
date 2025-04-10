#ifndef Telemetry_H
#define Telemetry_H

#include <SPI.h>
#include <LoRa.h>

void sendTelemetry(float altitude, float velocity, float pitch, float roll, float yaw);

#endif