#pragma once

#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#define ENABLE_ESP_SSLCLIENT

#include <FirebaseClient.h>
#include "main.h"

bool Firebase_IsReady();
void Firebase_setup();
void Firebase_loop();
void Firebase_processResponse(AsyncResult &aResult);
// void processData(AsyncResult &aResult);
void Firebase_Update_Sensor_Data(unsigned long timestamp, const SensorDataPayload &sensors);
void Firebase_Update_Relay_State(uint8_t kanal, bool state);
void Firebase_Update_INA_Data(unsigned long timestamp, const INA3221_DataPayload &data);

extern unsigned long Interval_mS;
extern bool firebaseUpdatePending;
extern ChannelUpdateData pendingUpdateData;
extern uint8_t currentChannelInProcess;
extern void Sensor_OnFirebaseResponse(bool success);