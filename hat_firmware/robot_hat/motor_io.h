#pragma once
static void drainMotorInput(){for(uint16_t i=0;i<128&&Serial1.available();i++)Serial1.read();}
static bool motorWrite(const uint8_t *data,size_t n){if(Serial1.availableForWrite()<static_cast<int>(n))return false;return Serial1.write(data,n)==n;}
static bool sendMotorMode(uint8_t id,uint8_t mode){
  const uint8_t frame[10]={id,0xA0,0,0,0,0,0,0,0,mode};drainMotorInput();
  if(!motorWrite(frame,sizeof(frame)))return false;
  // 10 bytes at 115200 complete within 1 ms; mode-write inter-command
  // separation is 4 ms. No unbounded Serial.flush on the control path.
  delay(4);return true;
}
static void updatePosition(Wheel &w,uint16_t raw,uint32_t now){
  if(!w.positionValid||now-w.lastPositionMs>cfg.feedbackMs){w.positionUnwrapped=raw;w.holdAnchor=raw;w.holdIntegral=0;}
  else w.positionUnwrapped+=static_cast<int16_t>(raw-w.lastPositionRaw);
  w.positionRaw=w.lastPositionRaw=raw;w.lastPositionMs=now;w.positionValid=true;
}
static MotorResult motorTransaction(uint8_t id,bool info,int16_t command,uint8_t expectedMode,bool allowAbort){
  uint8_t packet[10]={id,static_cast<uint8_t>(info?0x74:0x64),0,0,0,0,0,0,0,0};
  if(!info){packet[2]=static_cast<uint8_t>(command>>8);packet[3]=static_cast<uint8_t>(command);}packet[9]=crc8Maxim(packet,9);
  drainMotorInput();if(!motorWrite(packet,sizeof(packet)))return MOTOR_NO_REPLY;
  uint8_t reply[10];uint8_t received=0;bool bad=false;const uint32_t began=micros();
  while(static_cast<uint32_t>(micros()-began)<MOTOR_REPLY_TIMEOUT_US){
    pollHost();if(allowAbort&&stopPending)return MOTOR_INTERRUPTED;
    if(motionState()&&haveTargets&&millis()-lastTargetMs>=cfg.watchdogMs){trip(COMMAND_TIMEOUT);return MOTOR_INTERRUPTED;}
    for(uint16_t read=0;read<128&&Serial1.available();read++){
      const int value=Serial1.read();if(value<0)break;reply[received++]=static_cast<uint8_t>(value);if(received!=10)continue;
      if(crc8Maxim(reply,9)==reply[9]&&reply[0]==id&&reply[1]>=1&&reply[1]<=3){
        Wheel &w=wheel[id-1];const uint32_t now=millis();w.mode=reply[1];w.currentMa=clampS16(static_cast<int32_t>(static_cast<int16_t>((reply[2]<<8)|reply[3]))*8000/32767,-8000,8000);
        w.rpm=static_cast<int16_t>((reply[4]<<8)|reply[5]);w.error=reply[8];w.lastFeedbackMs=now;w.valid=true;
        if(info){w.tempC=reply[6];w.lastInfoMs=now;w.temperatureValid=true;}
        else updatePosition(w,static_cast<uint16_t>((reply[6]<<8)|reply[7]),now);
        if(expectedMode&&w.mode!=expectedMode)return MOTOR_WRONG_MODE;return MOTOR_OK;
      }
      bad=true;memmove(reply,reply+1,9);received=9;
    }delayMicroseconds(50);
  }return bad?MOTOR_BAD_REPLY:MOTOR_NO_REPLY;
}
static MotorResult sendCurrentMa(uint8_t id,int16_t ma,uint8_t expectedMode,bool allowAbort){
  const int16_t bounded=clampS16(ma,-HARD_MAX_CURRENT_MA,HARD_MAX_CURRENT_MA);
  return motorTransaction(id,false,clampS16(static_cast<int32_t>(bounded)*32767/8000,-32767,32767),expectedMode,allowAbort);
}
static FaultCode motorFailure(MotorResult r){return r==MOTOR_NO_REPLY?MOTOR_TIMEOUT:(r==MOTOR_BAD_REPLY?BAD_MOTOR_FRAME:MOTOR_FAULT);}
