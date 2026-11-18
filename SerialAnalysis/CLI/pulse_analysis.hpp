#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

struct Pulse {
  struct Sample { double t, red, ir; };
  std::vector<Sample> data;
  int bpm=0, spo2=0; bool validHr=false, validSpo2=false;
  void add(double t, uint32_t red, uint32_t ir) {
    data.push_back({t,double(red),double(ir)});
    while(!data.empty() && t-data.front().t>5.0) data.erase(data.begin());
  }
  void calculate() {
    validHr=validSpo2=false;
    if(data.size()<75 || data.back().t-data.front().t<3.0) return;
    double mr=0,mi=0; for(auto s:data){mr+=s.red;mi+=s.ir;} mr/=data.size();mi/=data.size();
    if(mr<1000 || mi<1000) return;
    double vr=0,vi=0; for(auto s:data){vr+=(s.red-mr)*(s.red-mr);vi+=(s.ir-mi)*(s.ir-mi);}
    vr=std::sqrt(vr/data.size());vi=std::sqrt(vi/data.size());
    if(vi<mi*0.002 || vr<mr*0.001) return;
    double ratio=(vr/mr)/(vi/mi);
    double saturation=110-25*ratio;
    if(saturation>=70 && saturation<=100) {spo2=int(std::lround(saturation));validSpo2=true;}
    std::vector<double> peaks;
    for(size_t i=2;i+2<data.size();++i) {
      double x=data[i].ir;
      if(x>mi+vi*0.25 && x>data[i-1].ir && x>=data[i+1].ir && x>data[i-2].ir && x>=data[i+2].ir &&
         (peaks.empty() || data[i].t-peaks.back()>0.33)) peaks.push_back(data[i].t);
    }
    if(peaks.size()>=3) { double period=(peaks.back()-peaks.front())/(peaks.size()-1);
      int candidate=int(std::lround(60/period)); if(candidate>=40 && candidate<=180) {bpm=candidate;validHr=true;} }
  }
};
