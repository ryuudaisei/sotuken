#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "crane.h"

// Runge-Kutta Method (4th order)
float f1(RK s, IN u)
{
    return s.x2;
}

float f2(RK s, IN u)
{
    return (float)-u.ft*(s.x1-R*cos(u.th));
}

float f3(RK s, IN u)
{
    return s.x4;
}

float f4(RK s, IN u)
{
    return (float)-u.ft*(s.x3-R*sin(u.th));
}

float f5(RK s, IN u)
{
    return s.x6;
}

float f6(RK s, IN u)
{
    return (float) -Grav-u.ft*(s.x5-Ht);
}

//rotary crane system
RK runge_kutta(RK s, IN u)
{
    RK tmp;
    float A1, B1, C1, D1;
    float A2, B2, C2, D2;
    float A3, B3, C3, D3;
    float A4, B4, C4, D4;
    float A5, B5, C5, D5;
    float A6, B6, C6, D6;

    tmp=s;
    A1=STEP*f1(tmp, u);     A2=STEP*f2(tmp, u);
    A3=STEP*f3(tmp, u);     A4=STEP*f4(tmp, u);
    A5=STEP*f5(tmp, u);     A6=STEP*f6(tmp, u);

    tmp.x1=s.x1+A1/2.0;     tmp.x2=s.x2+A2/2.0;
    tmp.x3=s.x3+A3/2.0;     tmp.x4=s.x4+A4/2.0;
    tmp.x5=s.x5+A5/2.0;     tmp.x6=s.x6+A6/2.0;
    B1=STEP*f1(tmp, u);     B2=STEP*f2(tmp, u);
    B3=STEP*f3(tmp, u);     B4=STEP*f4(tmp, u);
    B5=STEP*f5(tmp, u);     B6=STEP*f6(tmp, u);

    tmp.x1=s.x1+B1/2.0;     tmp.x2=s.x2+B2/2.0;
    tmp.x3=s.x3+B3/2.0;     tmp.x4=s.x4+B4/2.0;
    tmp.x5=s.x5+B5/2.0;     tmp.x6=s.x6+B6/2.0;
    C1=STEP*f1(tmp, u);     C2=STEP*f2(tmp, u);
    C3=STEP*f3(tmp, u);     C4=STEP*f4(tmp, u);
    C5=STEP*f5(tmp, u);     C6=STEP*f6(tmp, u);

    tmp.x1=s.x1+C1;  tmp.x2=s.x2+C2;
    tmp.x3=s.x3+C3;  tmp.x4=s.x4+C4;
    tmp.x5=s.x5+C5;  tmp.x6=s.x6+C6;
    D1=STEP*f1(tmp, u);     D2=STEP*f2(tmp, u);
    D3=STEP*f3(tmp, u);     D4=STEP*f4(tmp, u);
    D5=STEP*f5(tmp, u);     D6=STEP*f6(tmp, u);

    s.x1+=(A1+2.0*B1+2.0*C1+D1)/6.0;
    s.x2+=(A2+2.0*B2+2.0*C2+D2)/6.0;
    s.x3+=(A3+2.0*B3+2.0*C3+D3)/6.0;
    s.x4+=(A4+2.0*B4+2.0*C4+D4)/6.0;
    s.x5+=(A5+2.0*B5+2.0*C5+D5)/6.0;
    s.x6+=(A6+2.0*B6+2.0*C6+D6)/6.0;

    return s;
}