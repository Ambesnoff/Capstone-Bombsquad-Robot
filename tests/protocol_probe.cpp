// Independent native layout/CRC/validation probe for the checked v2 schema.
#include <cstdio>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "hat_firmware/robot_hat/protocol_v2.h"
using namespace ProtocolV2;
static uint16_t crc(const std::vector<uint8_t> &data) {
  uint16_t value=0xffff;
  for(uint8_t byte:data) { value ^= uint16_t(byte)<<8;
    for(unsigned bit=0;bit<8;bit++) value=(value&0x8000)?uint16_t((value<<1)^0x1021):uint16_t(value<<1);
  }
  return value;
}
static void frame(FrameType kind,uint16_t seq,const void *payload,size_t size) {
  std::vector<uint8_t> body={VERSION,uint8_t(kind),uint8_t(seq),uint8_t(seq>>8),uint8_t(size)};
  if(size) body.insert(body.end(),static_cast<const uint8_t*>(payload),static_cast<const uint8_t*>(payload)+size);
  uint16_t checksum=crc(body);std::cout<<"a55a";
  for(uint8_t byte:body) std::printf("%02x",byte);
  std::printf("%02x%02x\n",uint8_t(checksum),uint8_t(checksum>>8));
}
int main(int argc,char **argv) {
  if(argc>1 && std::string(argv[1])=="golden") {
    uint64_t nonce=0x0102030405060708ULL;
    frame(FrameType::HELLO,0x1234,&nonce,sizeof(nonce));
    frame(FrameType::STOP,65535,nullptr,0);
    TargetsPayload targets{{0x1020304050607080ULL,123},{-1225,1225,1225,-1225},uint8_t(Profile::BOOST)};
    frame(FrameType::TARGETS,65535,&targets,sizeof(targets));
    ConfigPayload config{{0x1020304050607080ULL,123},0,defaultConfig()};
    // Independent IEEE CRC32, canonical body identity.
    uint32_t identity=0xffffffff;const uint8_t *data=reinterpret_cast<const uint8_t*>(&config.config);
    for(size_t i=0;i<sizeof(Config);i++) {identity^=data[i];for(int bit=0;bit<8;bit++)identity=(identity>>1)^((identity&1)?0xedb88320:0);}
    config.config_id=(identity^0xffffffff)?identity^0xffffffff:1;
    frame(FrameType::CONFIG,2,&config,sizeof(config));
    StatusPayload status{};auto &h=status.header;
    h.boot_id=0x1020304050607080ULL;h.host_session=123;h.capabilities=255;
    h.build_id=0x12345678;h.accepted_seq=22;h.applied_seq=22;
    h.state=uint8_t(HatState::DISARMED);h.stop_state=uint8_t(StopState::CONFIRMED);
    h.sweep_us=18000;h.command_age_ms=8;h.boost_capacity_ms=20000;
    h.boost_remaining_ms=12500;h.boost_refill_remaining_ms=22500;h.hello_nonce=11;
    for(auto &w:status.wheels) {w.target_centi_rpm=-1250;w.rpm=0;w.current_ma=-180;
      w.position_raw=32000;w.temp_c=25;w.effective_cap_ma=800;w.hold_cap_ma=300;
      w.age_ms=4;w.temp_age_ms=400;w.mode=2;w.validity=15;}
    frame(FrameType::STATUS,42,&status,sizeof(status));
    return 0;
  }
  std::string line;
  while(std::getline(std::cin,line)) {
    if(line.size()!=2*sizeof(Config)) {std::cout<<"0\n";continue;}
    Config config{};uint8_t *bytes=reinterpret_cast<uint8_t*>(&config);
    for(size_t i=0;i<sizeof(Config);i++) bytes[i]=uint8_t(std::stoul(line.substr(2*i,2),nullptr,16));
    std::cout<<(validConfig(config)?"1":"0")<<'\n';
  }
}
