#include <stdio.h> 
#include <stdlib.h> 
#include <math.h> 

#include "crane.h" 
#include "nn.h"

// my file strucure (myfopen. c)
extern FILE *myfopen(char *fname, char *mode);

// rotary crane system (crane_3dmodel.c) 
extern RK runge_kutta(RK s, IN u); 

// three layered neural network controller (neural_network.c) 
extern NEURON_o neural_network(NEURON_i nin, NEURON_w w);

float crane_control_simulation(NEURON_w NCw) 
{
    float t, Eeb=0.0, Vel2=0.0, Dst2=0.0; 
    RK rk; 
    IN in; 
    NEURON_i NCi; 
    NEURON_o NCo; 

    // Initial condition 
    // crane system, evaluation function, NC 
    t=0.0;
    rk.x1=R*cos(THETA)+Leng*sin(BETA)*cos(THETA+ALPHA); 
    rk.x2=0.0; 
    rk.x3=R*sin(THETA)+Leng*sin(BETA)*sin(THETA+ALPHA); 
    rk.x4=0.0; 
    rk.x5=Ht-Leng*cos(BETA); 
    rk.x6=0.0; 
    Eeb=0.0; 
    // control simulation start
    while(t<=TIME){ 
        // input to controller 
        NCi.x_i[0] = Xref - rk.x1; 
        NCi.x_i[1] = dXref - rk.x2; 
        NCi.x_i[2] = Yref - rk.x3; 
        NCi.x_i[3] = dYref - rk.x4; 
        // Neural network controller 
        NCo = neural_network(NCi, NCw); 
        // from centrifugal force 
        Dst2 = rk.x1*rk.x1+rk.x3*rk.x3; 
        Vel2 = rk.x2*rk.x2+rk.x4*rk.x4; 
        // tension force divided by rope length 
        //3D model 
        //in.ft = sqrt(Vel2*Vel2/Dst2+Grav*Grav)/Leng; 
        // 2D model 
        in.ft = Grav/Leng; 
        // input to rotary crane system 
        if (t==0.0) 
            in.th = THETA; 
        else 
            in.th = NCo.y_k[0]; 
        // rotary crane model 
        rk = runge_kutta(rk, in); 
        // evaluation   
        if (t>=Ts && t<=Te){ 
            Eeb += ( ( Xref-rk.x1)*( Xref-rk.x1)+ (dXref-rk.x2)*(dXref-rk.x2)+ ( Yref-rk.x3)*( Yref-rk.x3)+ (dYref-rk.x4)*(dYref-rk.x4) ); 
        }
        t += STEP; 
    } // end crane system control 

    return Eeb;
}

float crane_control_simulation_data(NEURON_w NCw)
{
    FILE *fp1;
    float t, Eeb=0.0, Vel2=0.0, Dst=0.0;
    RK rk;
    IN in;
    NEURON_i NCi;
    NEURON_o NCo;

    fp1=myfopen("res01. dat", "w");
    // Initial sondition
    // crane system, evaluation function, NC
    t = 0.0;
    rk.x1=R*cos(THETA)+Leng*sin(BETA)*cos(THETA+ALPHA);
    rk.x2=0.0;
    rk.x3=R*sin(THETA)+Leng*sin(BETA)*sin(THETA+ALPHA);
    rk.x4=0.0;
    rk.x5=Ht-Leng*cos(BETA);
    rk.x6=0.0;
    Eeb=0.0;
    fprintf(fp1, "%f %f %f %f %f %f %f %f¥n", t, rk.x1, rk.x2, rk.x3, rk.x4,
    rk.x5, rk.x6, in.th);
    // control simulation start
    while(t<=TIME) {
        // input to controller
        NCi.x_i[0] =  Xref - rk.x1;
        NCi.x_i[1] = dXref - rk.x2;
        NCi.x_i[2] =  Yref - rk.x3;
        NCi.x_i[3] = dYref - rk.x4;
        // Neural network controller
        NCo = neural_network(NCi, NCw);
        // from centrifugal force
        Dst = rk.x1*rk.x1+rk.x3*rk.x3;
        Vel2 = rk.x2*rk.x2+rk.x4*rk.x4;
        // tension force divided by rope length
        // 3D model
        // in. ft = sqrt(Vel2*Vel2/Dst2+Grav*Grav)/Leng;
        // 2D model
        in. ft = Grav/Leng;
        // input to rotary crane system
        if (t==0.0)
             in.th = THETA;
        else 
             in.th = NCo.y_k[0];
        // rotary crane model
        rk = runge_kutta(rk, in);
        // evaluation
        if (t>=Ts && t<=Te) {
            Eeb += ( ( Xref-rk.x1)*( Xref-rk.x1)+
                    (dXref - rk.x2)*(dXref - rk.x2)+
                    ( Yref - rk.x3)*( Yref - rk.x3)+
                    (dYref - rk.x4)*(dYref - rk.x4) );
        }
        t += STEP;
        fprintf(fp1, "%f %f %f %f %f %f %f %f¥n", t, rk.x1, rk.x2, rk.x3, rk.x4,
    rk.x5, rk.x6, in.th);
    } // end crane system control
    fclose(fp1);
    return Eeb;
}