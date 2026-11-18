#pragma once
#include <array>
#include <cmath>
#include <cstddef>

// Two biquads, 40 Hz low-pass at 200 Hz. The original sketch's state assignments
// are self-assignments; use explicit x/y histories here.
struct Biquad {
  double b0, b1, b2, a1, a2, x1=0, x2=0, y1=0, y2=0;
  Biquad(double B0,double B1,double B2,double A1,double A2): b0(B0),b1(B1),b2(B2),a1(A1),a2(A2) {}
  double run(double x) { double y=b0*x+b1*x1+b2*x2-a1*y1-a2*y2;
    x2=x1;x1=x;y2=y1;y1=y; return y; }
};
struct EcgFilter {
  Biquad a{24511874.0/268435456,49023748.0/268435456,24511874.0/268435456,-263375836.0/268435456,92986877.0/268435456};
  Biquad b{40044810.0/268435456,80089620.0/268435456,40044810.0/268435456,-143201402.0/268435456,34946300.0/268435456};
  std::array<double,201> taps{}, history{}; size_t cursor=0; bool ready=false;
  EcgFilter() { const double pi=std::acos(-1.0), wc=2*pi*0.5/200;
    for (int k=0;k<201;++k) { int m=k-100; taps[k]=(m==0 ? 1-wc/pi : -std::sin(wc*m)/(pi*m))*(0.54-0.46*std::cos(2*pi*k/200)); }
  }
  double run(double raw) { if (!ready) { a.x1=a.x2=a.y1=a.y2=b.x1=b.x2=b.y1=b.y2=raw; history.fill(raw); ready=true; }
    history[cursor]=b.run(a.run(raw)); double y=0;
    for(size_t k=0;k<201;++k) y+=taps[k]*history[(cursor+201-k)%201]; cursor=(cursor+1)%201; return y;
  }
};
