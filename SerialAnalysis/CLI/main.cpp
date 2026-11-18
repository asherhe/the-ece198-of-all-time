#include "acceleration_analysis.hpp"
#include "ecg_filter.hpp"
#include "protocol.hpp"
#include "pulse_analysis.hpp"
#include "serial_port.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
static std::atomic<bool> stopping{false};
static void stopSignal(int) { stopping = true; }
static std::string isoUtc() { auto now=std::chrono::system_clock::now(); std::time_t t=std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm,&t);
#else
  gmtime_r(&t,&tm);
#endif
  std::ostringstream s; s<<std::put_time(&tm,"%Y-%m-%dT%H:%M:%SZ"); return s.str(); }
int main(int argc,char** argv) {
  try {
    std::signal(SIGINT, stopSignal);
    constexpr const char* usage = "Usage: serial-analysis --port COM3 [--output sessions] [--duration-min 30]\n"
                                  "       serial-analysis --self-test\n";
    if(argc==2 && (std::string(argv[1])=="--help" || std::string(argv[1])=="-h")) {
      std::cout << usage; return 0;
    }
    if(argc==2 && std::string(argv[1])=="--self-test") {
      uint8_t frame[18]{};frame[0]=0xa5;frame[1]=0x5a;frame[2]=1;
      w16(frame+3,42);w32(frame+5,123456);frame[9]=1;w16(frame+10,4);
      w16(frame+12,512);w16(frame+14,600);w16(frame+16,crc16(frame+2,14));
      Decoder d;int accepted=0;
      d.feed(frame,5,[&](const Frame&){++accepted;});
      d.feed(frame+5,13,[&](const Frame& f){if(f.seq==42&&f.t==123456&&f.data.size()==4)++accepted;});
      frame[12]^=1;d.feed(frame,18,[&](const Frame&){++accepted;});
      EcgFilter e;double result=e.run(512);
      if(accepted!=1||d.bad!=1||!std::isfinite(result)) throw std::runtime_error("self-test failed");
      std::cout<<"self-test passed\n";return 0;
    }
    std::string portName, output="sessions"; int duration=30;
    for(int i=1;i<argc;++i) {std::string a=argv[i]; if(a=="--port"&&i+1<argc)portName=argv[++i];
      else if(a=="--output"&&i+1<argc)output=argv[++i]; else if(a=="--duration-min"&&i+1<argc)duration=std::stoi(argv[++i]);
      else throw std::runtime_error(usage);}
    if(portName.empty()) throw std::runtime_error(std::string("Missing --port.\n") + usage);
    if(duration<=0) throw std::runtime_error(std::string("--duration-min must be positive.\n") + usage);
    const std::string startedUtc=isoUtc();
    fs::create_directories(output); fs::path dir=fs::path(output)/startedUtc;
    std::string name=dir.filename().string(); std::replace(name.begin(),name.end(),':','-'); dir=fs::path(output)/name;
    fs::create_directories(dir);
    std::ofstream ecg(dir/"ecg.csv"),accel(dir/"acceleration.csv"),vitals(dir/"vitals.csv");
    ecg<<"board_us,diff_raw,wilson_raw,diff_filtered,wilson_filtered\n";
    accel<<"board_us,magnitude_m_s2,smoothed_m_s2\n";
    vitals<<"board_us,heart_rate_bpm,heart_rate_valid,spo2_percent,spo2_valid\n";
    Port port(portName); Decoder decoder; EcgFilter diff,wilson; Pulse pulse;
    auto start=Clock::now(), lastDisplay=start, lastStatus=start; bool haveSeq=false; uint16_t prevSeq=0;
    uint64_t gaps=0,frames=0,ecgSamples=0,accelSamples=0,opticalSamples=0;
    uint64_t epoch=0; uint32_t prevTime=0; bool haveTime=false; AccelerationAnalysis motion;
    auto unwrap=[&](uint32_t t)->uint64_t {if(haveTime&&t<prevTime&&prevTime-t>0x80000000u)epoch+=1ull<<32;
      prevTime=t;haveTime=true;return epoch+t;};
    std::cout<<"Recording to "<<dir<<"\n";
    while(!stopping && Clock::now()-start<std::chrono::minutes(duration)) {
      uint8_t buf[1024]; int n=port.read(buf,sizeof(buf));
      decoder.feed(buf,n,[&](const Frame& f) {
        int width=f.type==1?4:f.type==2?6:f.type==3?12:0;
        if(!width || f.count==0 || f.data.size()!=size_t(f.count*width)) return;
        if(haveSeq) gaps+=uint16_t(f.seq-prevSeq-1); prevSeq=f.seq;haveSeq=true;++frames;
        uint64_t t0=unwrap(f.t);
        for(int i=0;i<f.count;++i) {
          const uint8_t* p=f.data.data()+i*width;
          if(f.type==1) {uint64_t t=t0+uint64_t(i)*5000; double d=diff.run(u16(p)),w=wilson.run(u16(p+2));
            ecg<<t<<','<<u16(p)<<','<<u16(p+2)<<','<<d<<','<<w<<'\n';++ecgSamples;}
          else if(f.type==2) {uint64_t t=t0+uint64_t(i)*20000;
            auto result=motion.run(int16_t(u16(p)),int16_t(u16(p+2)),int16_t(u16(p+4)));
            accel<<t<<','<<result.magnitude<<','<<result.smoothed<<'\n';++accelSamples;}
          else {uint64_t t=unwrap(u32(p));pulse.add(t/1e6,u32(p+4),u32(p+8));++opticalSamples;}
        }
      });
      auto now=Clock::now();
      if(now-lastDisplay>=std::chrono::seconds(1)) {
        pulse.calculate(); uint32_t remaining=uint32_t(std::max<int64_t>(0,std::chrono::duration_cast<std::chrono::seconds>(std::chrono::minutes(duration)-(now-start)).count()));
        uint8_t frame[21]{};frame[0]=0xa5;frame[1]=0x5a;frame[2]=0x80;w32(frame+5,0);frame[9]=1;w16(frame+10,7);
        w16(frame+12,uint16_t(pulse.bpm));w32(frame+14,remaining);frame[18]=pulse.validHr?1:0;
        w16(frame+19,crc16(frame+2,17));port.write(frame,sizeof(frame));
        vitals<<(haveTime?epoch+prevTime:0)<<','<<pulse.bpm<<','<<pulse.validHr<<','<<pulse.spo2<<','<<pulse.validSpo2<<'\n';
        ecg.flush();accel.flush();vitals.flush();lastDisplay=now;
      }
      if(now-lastStatus>=std::chrono::seconds(10)) {std::cout<<"frames="<<frames<<" gaps="<<gaps<<" bad_crc="<<decoder.bad<<" ECG="<<ecgSamples<<" accel="<<accelSamples<<" optical="<<opticalSamples<<'\n';lastStatus=now;}
    }
    std::ofstream meta(dir/"metadata.json");meta<<"{\n  \"protocol_version\": 1,\n  \"started_utc\": \""<<startedUtc<<"\",\n  \"port\": \""<<portName<<"\",\n  \"duration_minutes\": "<<duration<<",\n  \"frames\": "<<frames<<",\n  \"sequence_gaps\": "<<gaps<<",\n  \"bad_frames\": "<<decoder.bad<<"\n}\n";
    std::cout<<"Finished: "<<dir<<'\n';
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
