#pragma once

#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#define ENABLE_ESP_SSLCLIENT

#include <FirebaseClient.h>
#include "main.h"
//#include "ExampleFunctions.h" // Provides the functions used in the examples.


void Firebase_setup();
void Firebase_loop();
void Firebase_processResponse(AsyncResult &aResult);
void Firebase_Update_Sensor_Data(unsigned long timestamp, const SensorDataPayload &sensors);
void Firebase_Update_Relay_State(int kanal, bool state);
void Firebase_Update_INA_Data(unsigned long timestamp, const INA3221_DataPayload &data);

extern unsigned long Interval_mS;
extern bool firebaseUpdatePending;
extern ChannelUpdateData pendingUpdateData;
extern uint8_t currentChannelInProcess;