/*
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2022 Sensnology AB
 * Full contributor list: https://github.com/mysensors/MySensors/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * -------------------------------------------------------------------------------
 *
 * Copyright (c) 2013, Majenko Technologies and S.J.Hoeksma
 * Copyright (c) 2015, LeoDesigner
 * https://github.com/leodesigner/mysensors-serial-transport
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of Majenko Technologies.
 ********************************************************************************/
// #include <stdint.h>
// #include <stddef.h>
// #include <stdbool.h>
// #define ARDUINO_ARCH_STM32

#if defined(ARDUINO_ARCH_ESP32)

#include "driver/twai.h"
#include "driver/gpio.h"

#elif defined(ARDUINO_ARCH_STM32)

#include "hal/transport/CAN/driver/STM32_CAN/src/STM32_CAN.h"
#include "hal/transport/CAN/driver/STM32_CAN/src/STM32_CAN.cpp"

#else
#include "hal/transport/CAN/driver/MCP_CAN_lib/mcp_can.h"
#include "hal/transport/CAN/driver/MCP_CAN_lib/mcp_can_dfs.h"
#include "hal/transport/CAN/driver/MCP_CAN_lib/mcp_can.cpp"
#endif

#if defined(MY_DEBUG_VERBOSE_CAN)
#define CAN_DEBUG(x,...) DEBUG_OUTPUT(x, ##__VA_ARGS__)
#else
#define CAN_DEBUG(x,...)
#endif

// Platform-specific CAN initialization
#if defined(ARDUINO_ARCH_ESP32)


#elif defined(ARDUINO_ARCH_STM32)

#else
MCP_CAN _MCP_CAN(MY_CAN_CS);
#endif

bool canInitialized = false;

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
unsigned char _nodeId;
uint8_t message_id = 0;

#define MY_CAN_PACKET_TIMEOUT 100

// ================= 配置参数 =================
#ifndef MAX_MESSAGE_SIZE
    #define MAX_MESSAGE_SIZE   (128)   // 最大消息长度 (32 / 64 / 128)
#endif

#define MY_CAN_MAX_FRAMES       (16)

typedef struct {
    uint8_t len;
    uint8_t data[MAX_MESSAGE_SIZE];
    bool packetReceived[MY_CAN_MAX_FRAMES];
    uint8_t address;
    uint8_t totalReceivedParts;
    bool locked;
    uint32_t timestamp;
    uint8_t packetId;
    uint8_t expectedParts;
    bool ready;
} CAN_Packet;

CAN_Packet packets[MY_CAN_BUF_SIZE];

// ===== Header functions =====
static inline uint32_t _buildHeader(uint8_t messageId, uint8_t totalParts, uint8_t currentPart, uint8_t to, uint8_t from)
{
    uint32_t header = 0;
    header |= ((uint32_t)messageId & 0xFF) << 21;
    header |= ((uint32_t)totalParts & 0x0F) << 17;
    header |= ((uint32_t)currentPart & 0x0F) << 13;
    header |= ((uint32_t)to & 0xFF) << 5;
    header |= ((uint32_t)from & 0x1F);
    return header;
}

static inline void _parseHeader(uint32_t header, uint8_t &messageId, uint8_t &totalParts, uint8_t &currentPart, uint8_t &to, uint8_t &from)
{
    messageId   = (header >> 21) & 0xFF;
    totalParts  = (header >> 17) & 0x0F;
    currentPart = (header >> 13) & 0x0F;
    to          = (header >> 5) & 0xFF;
    from        = header & 0x1F;
}

// ===== Buffer functions =====
void _cleanSlot(uint8_t slot)
{
    packets[slot].locked = false;
    packets[slot].len = 0;
    packets[slot].address = 0;
    packets[slot].totalReceivedParts = 0;
    packets[slot].expectedParts = 0;
    packets[slot].timestamp = 0;
    packets[slot].packetId = 0;
    packets[slot].ready = false;
    for(uint8_t i=0;i<MY_CAN_MAX_FRAMES;i++) packets[slot].packetReceived[i] = false;
}

uint8_t _findCanPacketSlot()
{
    uint8_t slot = MY_CAN_BUF_SIZE;
    uint8_t i;
    for(i=0;i<MY_CAN_BUF_SIZE;i++){
        if(!packets[i].locked) { slot=i; break; }
    }
    if(slot < MY_CAN_BUF_SIZE) return slot;
    // no empty slot, evict oldest
    slot = 0;
    for(i=1;i<MY_CAN_BUF_SIZE;i++){
        if(packets[i].timestamp < packets[slot].timestamp) slot = i;
    }
    CAN_DEBUG("!CAN:SLOT:FULL -> evict slot=%u\n", slot);
    _cleanSlot(slot);
    return slot;
}

uint8_t _findCanPacketSlot(long unsigned int from, long unsigned int currentPart, long unsigned int messageId)
{
    uint8_t slot = MY_CAN_BUF_SIZE;
    for(uint8_t i=0;i<MY_CAN_BUF_SIZE;i++){
        if(packets[i].locked && packets[i].address==from && packets[i].packetId==messageId && !packets[i].packetReceived[currentPart]){
            slot=i; break;
        }
    }
    return slot;
}

// ===== Transport functions =====
bool transportInit(void)
{
    CAN_DEBUG("CAN:INIT NodeID=%u\n", _nodeId);

#if defined(ARDUINO_ARCH_ESP32)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(MY_CAN_TX_PIN, MY_CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = {
        .brp = (uint32_t)(80000000UL/(MY_CAN_SPEED*16)), // 80MHz / (speed*16)
        .tseg_1 = 15, .tseg_2 = 8, .sjw = 3, .triple_sampling = false
    };
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if(twai_driver_install(&g_config, &t_config, &f_config)!=ESP_OK) return false;
    if(twai_start()!=ESP_OK) return false;
#elif defined(ARDUINO_ARCH_STM32)
if (!CANInit(MY_CAN_SPEED, 2)) {
    CAN_DEBUG("CAN:INIT Failed\n");
    return false;
}
if(!CANStart()){
    CAN_DEBUG("CAN:START Failed\n");
    return false;
}
#else
    if(_MCP_CAN.begin(MCP_STDEXT, MY_CAN_SPEED, MY_CAN_CLOCK)!=CAN_OK) return false;
#endif

    canInitialized = true;
    for(uint8_t i=0;i<MY_CAN_BUF_SIZE;i++) _cleanSlot(i);
    CAN_DEBUG("CAN:INIT:OK\n");
    return true;
}

bool transportSend(const uint8_t to, const void *data, const uint8_t len, const bool noACK)
{
    (void)noACK;
    const uint8_t *payload = static_cast<const uint8_t*>(data);
    uint8_t totalParts = (len + 7) / 8;
    message_id = (message_id + 1) & 0xFF;

    CAN_DEBUG("CAN:SND:Start id=%u to=%u parts=%u len=%u\n", message_id, to, totalParts, len);

    for(uint8_t part=0; part<totalParts; part++){
        uint8_t partLen = (part==totalParts-1)? (len%8) : 8;
        if(partLen==0) partLen=8;
        uint8_t buf[8]={0};
        memcpy(buf, payload + part*8, partLen);
        uint32_t header = _buildHeader(message_id, totalParts, part, to, _nodeId);

#if defined(ARDUINO_ARCH_ESP32)
        twai_message_t msg={};
        msg.identifier = header; msg.extd=1; msg.data_length_code=partLen;
        memcpy(msg.data, buf, partLen);
        if(twai_transmit(&msg,pdMS_TO_TICKS(100))!=ESP_OK){ CAN_DEBUG("!CAN:SND:FAIL part=%u\n",part); return false; }
#elif defined(ARDUINO_ARCH_STM32)
        CAN_msg_t msg={};
        msg.id = header; msg.len = partLen; msg.format = EXTENDED_FORMAT; msg.type = 0;
        memcpy(msg.data, buf, partLen);
 
        // CANSend(&msg);
        if (!CANSend(&msg))
        {
            CAN_DEBUG("!CAN:SND:FAIL part=%u\n", part);
            return false;
        }
        
#else
        if(_MCP_CAN.sendMsgBuf(header, partLen, buf)!=CAN_OK){ CAN_DEBUG("!CAN:SND:FAIL part=%u\n",part); return false; }
#endif
        CAN_DEBUG("CAN:SND:part=%u len=%u OK\n", part, partLen);
    }
    CAN_DEBUG("CAN:SND:Complete id=%u\n", message_id);
    return true;
}

bool transportDataAvailable(void)
{
#if defined(ARDUINO_ARCH_ESP32)
    twai_message_t msg;
    if(twai_receive(&msg,0)!=ESP_OK) return false;
    rxId=msg.identifier; len=msg.data_length_code; memcpy(rxBuf,msg.data,len);
#elif defined(ARDUINO_ARCH_STM32)
    CAN_msg_t msg={};
    if(!CANReceive(&msg)) return false;
        rxId = msg.id; 
        len = msg.len;
        memcpy(rxBuf, msg.data, len);
#else
    if(!hwDigitalRead(MY_CAN_INT) || _MCP_CAN.readMsgBuf(&rxId,&len,rxBuf)!=CAN_OK) return false;
#endif

    uint8_t from,to,partIdx,totalParts,msgId;
    _parseHeader(rxId,msgId,totalParts,partIdx,to,from);
    CAN_DEBUG("CAN:RCV:msgId=%u from=%u to=%u part=%u/%u len=%u\n", msgId, from, to, partIdx, totalParts, len);

    if(to!=_nodeId && to!=BROADCAST_ADDRESS) {
        CAN_DEBUG("CAN:RCV:Drop -> not for me (to=%u)\n", to);
        return false;
    }

    uint8_t slot=_findCanPacketSlot(from,partIdx,msgId);
    if(slot==MY_CAN_BUF_SIZE){
        slot=_findCanPacketSlot();
        _cleanSlot(slot);
        packets[slot].locked=true; packets[slot].address=from;
        packets[slot].packetId=msgId; packets[slot].expectedParts=totalParts;
        packets[slot].timestamp=millis();
        CAN_DEBUG("CAN:RCV:New slot=%u for msgId=%u\n", slot, msgId);
    }

    if(!packets[slot].packetReceived[partIdx]){
        memcpy(packets[slot].data+partIdx*8,rxBuf,len);
        packets[slot].len+=len;
        packets[slot].totalReceivedParts++;
        packets[slot].packetReceived[partIdx]=true;
        CAN_DEBUG("CAN:RCV:slot=%u stored part=%u len=%u totalParts=%u\n", slot, partIdx, len, packets[slot].totalReceivedParts);
    }

    if(millis()-packets[slot].timestamp>MY_CAN_PACKET_TIMEOUT){
        CAN_DEBUG("!CAN:RCV:TIMEOUT slot=%u\n",slot);
        _cleanSlot(slot);
        return false;
    }

    if(packets[slot].totalReceivedParts>=packets[slot].expectedParts){
        packets[slot].ready=true;
        CAN_DEBUG("CAN:RCV:slot=%u complete len=%u\n", slot, packets[slot].len);
        return true;
    }
    return false;
}

uint8_t transportReceive(void *data)
{
    uint8_t slot = MY_CAN_BUF_SIZE;
    for(uint8_t i=0;i<MY_CAN_BUF_SIZE;i++){
        if(packets[i].ready){ slot=i; break; }
    }
    if(slot<MY_CAN_BUF_SIZE){
        memcpy(data,packets[slot].data,packets[slot].len);
        uint8_t retLen = packets[slot].len;
        CAN_DEBUG("CAN:RX:Deliver slot=%u len=%u\n", slot, retLen);
        _cleanSlot(slot);
        return retLen;
    }
    return 0;
}

void transportSetAddress(const uint8_t address){ _nodeId=address; CAN_DEBUG("CAN:ADDR set=%u\n",address); }
uint8_t transportGetAddress(void){ return _nodeId; }
bool transportSanityCheck(void){ return true; }

void transportPowerDown(void){
#if defined(ARDUINO_ARCH_ESP32) 
    twai_stop();
#elif defined(ARDUINO_ARCH_STM32) 
    CANDeinit();
#else 
    _MCP_CAN.setMode(MCP_SLEEP);
#endif
    CAN_DEBUG("CAN:PowerDown\n");
}

void transportPowerUp(void){
#if defined(ARDUINO_ARCH_ESP32)
    twai_start();
#elif defined(ARDUINO_ARCH_STM32)
    CANStart();
#else
    _MCP_CAN.setMode(MCP_NORMAL);
#endif
    CAN_DEBUG("CAN:PowerUp\n");
}

// 未实现 / 保留
void transportSleep(void){ CAN_DEBUG("CAN:Sleep\n"); }
void transportStandBy(void){ CAN_DEBUG("CAN:StandBy\n"); }
int16_t transportGetSendingRSSI(void){ return INVALID_RSSI; }
int16_t transportGetReceivingRSSI(void){ return INVALID_RSSI; }
int16_t transportGetSendingSNR(void){ return INVALID_SNR; }
int16_t transportGetReceivingSNR(void){ return INVALID_SNR; }
int16_t transportGetTxPowerPercent(void){ return 100; }
int16_t transportGetTxPowerLevel(void){ return 100; }
bool transportSetTxPowerPercent(const uint8_t powerPercent){ (void)powerPercent; return false; }
