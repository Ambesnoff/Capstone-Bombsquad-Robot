#pragma once
#include "build_identity.h"
static uint64_t helloNonce=0,sessionCounter=0;
static void sendStatus() {
  ProtocolV2::StatusPayload payload{};auto &h=payload.header;const uint32_t now=millis();
  h.boot_id=bootId;h.host_session=hostSession;h.capabilities=255;h.build_id=ROBOT_BUILD_ID;h.config_id=configId;
  h.accepted_seq=lastAcceptedSequence;h.applied_seq=lastAppliedSequence;h.accepted_ms=lastAcceptedMs;h.applied_ms=lastAppliedMs;
  h.state=state;h.fault_code=faultCode;h.stop_state=stopState;h.requested_profile=requestedProfile;h.applied_profile=appliedProfile;h.config_result=configResult;
  h.reason_flags=reasonFlags;h.sweep_us=lastSweepUs;h.command_age_ms=haveTargets?saturate16(now-lastTargetMs):65535;
  h.boost_remaining_ms=static_cast<uint32_t>(boostRemainingMs);h.boost_capacity_ms=cfg.boostCapacityMs;
  h.boost_refill_remaining_ms=static_cast<uint32_t>((cfg.boostCapacityMs-boostRemainingMs)*cfg.boostRefillMs/cfg.boostCapacityMs);
  h.cooldown_remaining_ms=coolSinceMs?static_cast<uint32_t>(max(0,static_cast<int>(cfg.cooldownMs)-static_cast<int>(now-coolSinceMs))):cfg.cooldownMs;
  h.hold_flags=(state==HOLDING?1:0)|((reasonFlags&R_HOLD_LIMITED)?2:0)|(disarmedHolding?4:0);h.fault_wheel=faultWheel;h.hello_nonce=helloNonce;
  for(uint8_t i=0;i<WHEEL_COUNT;i++){const Wheel &w=wheel[i];auto &s=payload.wheels[i];
    s.target_centi_rpm=targetCentiRpm[i];s.rpm=w.rpm;s.current_ma=w.currentMa;s.position_raw=w.positionRaw;s.temp_c=w.tempC;
    s.effective_cap_ma=static_cast<uint16_t>(w.effectiveCapMa);s.hold_cap_ma=static_cast<uint16_t>(w.holdCapMa);
    s.age_ms=w.valid?saturate16(now-w.lastFeedbackMs):65535;s.temp_age_ms=w.temperatureValid?saturate16(now-w.lastInfoMs):65535;
    s.error=w.error;s.mode=w.mode;s.validity=(w.valid?3:0)|(w.positionValid&&now-w.lastPositionMs<=cfg.feedbackMs?4:0)|(w.temperatureValid?8:0);s.reason_flags=w.reason;
  }
  uint8_t frame[9+sizeof(payload)]={0xA5,0x5A,2,STATUS_FRAME};put16(frame+4,statusSequence++);frame[6]=sizeof(payload);
  memcpy(frame+7,&payload,sizeof(payload));put16(frame+7+sizeof(payload),crc16(frame+2,5+sizeof(payload)));
  // Drop a status snapshot if UART space is exhausted. Never block control on
  // a disconnected host or telemetry flood. Later snapshots replace it.
  if(Serial.availableForWrite()>=static_cast<int>(sizeof(frame)))Serial.write(frame,sizeof(frame));
  lastStatusMs=now;
}
static ControllerConfig unpackConfig(const uint8_t *data) {
  ControllerConfig c;
  uint16_t *fields[]={&c.maxRpm,&c.maxCurrentMa,&c.neutralBrakeMa,&c.accelRpmS,&c.decelRpmS,&c.kpMaPerRpm,&c.kiMaPerRpmS,&c.ffMaPerRpmS,&c.watchdogMs,&c.periodMs,&c.stallMs,&c.gentleMa,&c.normalMa,&c.boostMa,&c.boostCapacityMs,&c.boostRefillMs,&c.tempPollMs,&c.boostFreshMs,&c.tempFreshMs,&c.cooldownMs,&c.capRampMaS,&c.holdMa,&c.holdKp,&c.holdKi,&c.holdDamping,&c.settleMs,&c.feedbackMs,&c.stallTargetCenti,&c.stallSpeedCenti,&c.stallCurrentMa,&c.abnormalCurrentMa,&c.abnormalMs,&c.abnormalMarginMa,&c.saturationWarnMs,&c.stopVerifyMs};
  for(uint8_t i=0;i<35;i++)*fields[i]=u16(data+2*i);
  uint8_t *bytes[]={&c.tempWarnC,&c.tempDerateC,&c.tempLimitC,&c.tempReleaseC,&c.tempHysteresisC,&c.holdEnabled,&c.disarmedHoldEnabled,&c.stallEnabled};
  for(uint8_t i=0;i<8;i++)*bytes[i]=data[70+i];return c;
}
static uint32_t configCrc32(const uint8_t *data,size_t n) {uint32_t crc=0xFFFFFFFFU;for(size_t i=0;i<n;i++){crc^=data[i];for(uint8_t b=0;b<8;b++)crc=(crc>>1)^((crc&1)?0xEDB88320U:0);}return (crc^0xFFFFFFFFU)?(crc^0xFFFFFFFFU):1;}
static void requestStop() {haveTargets=false;armPending=false;stopState=1;if(!stopping)stopPending=true;state=faultCode==NO_FAULT?STOPPING:FAULT;}
static void handleHostFrame(uint8_t type,uint16_t seq,const uint8_t *data,uint8_t n) {
  if(type==HELLO&&n==8){
    helloNonce=u64(data);++sessionCounter;hostSession=bootId^sessionCounter;if(!hostSession)hostSession=bootId^(++sessionCounter);
    // The HAT issues a never-reused token each HELLO. Replaying HELLO can
    // inhibit motion, but never makes captured commands valid again.
    configured=false;configId=0;configResult=0;acceptCommand(seq);requestStop();sendStatus();return;
  }
  if(type==STOP&&n==0){if(freshCommand(seq))acceptCommand(seq);requestStop();sendStatus();return;}
  // STATUS_REQ is observational and cannot advance the motion boundary.
  if(type==STATUS_REQ&&n==0){sendStatus();return;}
  if(n<16||!hostSession||u64(data)!=bootId||u64(data+8)!=hostSession){reasonFlags|=R_SESSION;configResult=4;sendStatus();return;}
  if(!freshCommand(seq))return;
  if(type==CONFIG&&n==sizeof(ProtocolV2::ConfigPayload)){
    if(state!=DISARMED||stopPending||stopping||stopState!=3||!stationaryFresh(true)){configResult=2;reasonFlags|=R_CONFIG;sendStatus();return;}
    const ControllerConfig candidate=unpackConfig(data+20);const uint32_t id=u32(data+16);
    if(!validConfig(candidate)||!id||id!=configCrc32(data+20,sizeof(ProtocolV2::Config))){configResult=3;reasonFlags|=R_CONFIG;sendStatus();return;}
    cfg=candidate;configured=true;configId=id;configResult=1;boostRemainingMs=min(boostRemainingMs,static_cast<float>(cfg.boostCapacityMs));acceptCommand(seq);markApplied(seq);sendStatus();return;
  }
  if(stopPending||stopping)return;
  if(type==ARM&&n==sizeof(ProtocolV2::ArmPayload)&&configured&&u32(data+16)==configId&&faultCode==NO_FAULT&&
    ((state==DISARMED&&stopState==3&&stationaryFresh(true))||(state==HOLDING&&disarmedHolding&&stationaryFresh(true)))){
      armSequence=seq;armPending=true;return;
  }
  if(type==TARGETS&&n==sizeof(ProtocolV2::TargetsPayload)&&motionState()){
    if(data[24]>2){trip(CONFIGURATION_FAULT);return;}
    int16_t requested[WHEEL_COUNT];for(uint8_t i=0;i<WHEEL_COUNT;i++){requested[i]=s16(data+16+2*i);if(abs(static_cast<int>(requested[i]))>cfg.maxRpm*100){trip(CONFIGURATION_FAULT);return;}}
    memcpy(stagedTargetCentiRpm,requested,sizeof(requested));stagedProfile=data[24];stagedSequence=seq;lastTargetMs=millis();haveTargets=true;acceptCommand(seq);
  }
}
static void pollHost() {
  for(uint16_t count=0;count<128&&Serial.available();count++){
    const int value=Serial.read();if(value<0)break;
    if(hostRxLength==sizeof(hostRx)){memmove(hostRx,hostRx+1,--hostRxLength);}hostRx[hostRxLength++]=static_cast<uint8_t>(value);
    while(hostRxLength>=2){
      if(hostRx[0]!=0xA5||hostRx[1]!=0x5A){memmove(hostRx,hostRx+1,--hostRxLength);continue;}
      if(hostRxLength<7)break;const uint8_t length=hostRx[6];
      if(hostRx[2]!=2||length>HOST_MAX_PAYLOAD){memmove(hostRx,hostRx+1,--hostRxLength);continue;}
      const uint16_t total=9+length;if(hostRxLength<total)break;
      if(crc16(hostRx+2,5+length)!=u16(hostRx+7+length)){memmove(hostRx,hostRx+1,--hostRxLength);continue;}
      uint8_t data[HOST_MAX_PAYLOAD];if(length)memcpy(data,hostRx+7,length);const uint8_t type=hostRx[3];const uint16_t seq=u16(hostRx+4);
      hostRxLength-=total;memmove(hostRx,hostRx+total,hostRxLength);handleHostFrame(type,seq,data,length);
    }
  }
}
