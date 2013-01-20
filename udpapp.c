/******************************************************************************
 * 
 * Filename:  	udpapp.h
 * Description:	UDP app for the WiShield 1.0
 * 
 ******************************************************************************
 * 
 * TCP/IP stack and driver for the WiShield 1.0 wireless devices
 * 
 * Copyright(c) 2009 Async Labs Inc. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 * Contact Information:
 * <asynclabs@asynclabs.com>
 * 
 * Author               Date        Comment
 * ---------------------------------------------------------------
 * AsyncLabs			07/11/2009	Initial version
 * Oly Leung      20/01/2013  WifiShield_OSC_i2c
 * 
 *****************************************************************************/

#include "config.h"
#include "udpapp.h"
#include "uip-conf.h"
#include "uip.h"
#include <string.h>


static struct udpapp_state s;
static struct uip_udp_conn* connOutgoing;

#define STATE_INIT				0
#define STATE_LISTENING         1
#define STATE_HELLO_RECEIVED	2
#define STATE_NAME_RECEIVED		3
const void *lastData;
int lastSize;
//static struct udpapp_state s;

void dummy_app_appcall(void)
{
}

void udpapp_init(void)
{
  lastSize = 0;
  uip_ipaddr_t addr;
  struct uip_udp_conn *c;

  //address of Ubuntu box (or send to address)
  uip_ipaddr(&addr, 192, 168, 1, 100);
  c = uip_udp_new(&addr, HTONS(6660));
  if(c != NULL) 
  {
    uip_udp_bind(c, HTONS(1234));
  }
}



static void send_data(void)
{
  memcpy(uip_appdata, lastData, lastSize);
  uip_send(uip_appdata, lastSize);
}

void udpapp_appcall(void)
{
  if(0 != uip_poll()) 
  {
    send_data();
  }
}
void sendOSC(uint8_t *msgData, int msgSize)
{
  lastSize = msgSize;
  lastData = msgData;
}
