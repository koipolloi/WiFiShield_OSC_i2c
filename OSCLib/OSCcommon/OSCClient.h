/*
 
 ArdOSC 2.1 - OSC Library for Arduino.
 
 -------- Lisence -----------------------------------------------------------
 
 ArdOSC
 
 The MIT License
 
 Copyright (c) 2009 - 2011 recotana( http://recotana.com )　All right reserved
 
 */

#ifndef ArdOSC_OSCClient2_h
#define ArdOSC_OSCClient2_h


#include "OSCcommon.h"
#include "OSCMessage.h"
#include "OSCEncoder.h"

//#include "../WiFlyHQ/WiFlyHQ.h"

#define kDummyPortNumber 10000


class OSCClient{
    
private:

//    WiFly* wiFly;

    OSCEncoder::OSCEncoder encoder;
	long lastSendMillis;	///< used to make sure that the Wifly sends each message in a single package.
	int TimeoutMillis;	///< if we wait that long between messages, an UDP package is sent.
public:

    //OSCClient(void)//WiFly* wiFly);
    ~OSCClient(void);
    uint8_t* send(OSCMessage *_message);
//	uint8_t sendInt(int value, char* adress);	 ///< send a single number to the specified adress conveniently
//	uint8_t sendFloat(float value, char* adress);///< send a single number to the specified adress conveniently
};


#endif
