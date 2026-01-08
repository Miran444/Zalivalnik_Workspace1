#pragma once

#define ENABLE_USER_AUTH
#define ENABLE_DATABASE
#define ENABLE_ESP_SSLCLIENT

#include <FirebaseClient.h>
#include "main.h"
//#include "ExampleFunctions.h" // Provides the functions used in the examples.

void processData(AsyncResult &aResult);
void Firebase_setup();
void Firebase_loop();
void Firebase_processResponse(AsyncResult &aResult);

extern unsigned long Interval_mS;
extern bool firebaseUpdatePending;
extern ChannelUpdateData pendingUpdateData;
extern uint8_t currentChannelInProcess;