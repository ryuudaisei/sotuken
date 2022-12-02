/*
######################################## 
### Cuckoo search method
### CS method  
### programmed by nakazono. [2021/09/17] 
######################################## */

#include<stdio.h> 
#include<stdlib.h> 
#include<math.h>

#include "crane.h" 
#include "nn.h" 
#include "bat.h"

//normal random number
float rand_n(void)
{
    return sqrt(-2.0*log(drand48()))*cos(2.0*PI*drand48());
}

float sigma(float beta)
{
    float nume,denom;
    nume = tgamma(1.0+beta)*sin(PI*beta/2.0);
    denom = tgamma((1.0+beta)/2.0)*beta*pow(2.0,
    (beta-1.0)/2);
             return pow(nume/denom,1.0/beta);
}

//initial weights　　型が構造体　　initがNC１コ　rand_wが全体
INDIVIDUAL rand_w(void)
{
    int i,j,k;
    INDIVIDUAL init;

    for(j=0;j<J_MAX;j++){
        for(i=0;i<I_MAX;i++){

    init.whid[i][j]=(float)(drand48()*(2.0*XMAX)-XMAX);
                          }
                }
                for(k=0;k<K_MAX;k++){
                         for(j=0;j<J_MAX;j++){
    init.whid[i][k]=(float)(drand48()*(2.0*XMAX)-XMAX);
                          }
                }

                return init;
}

//selection(ranking method) kakkoのアルゴリズム
void sort(INDIVIDUAL kotai[IND_SIZE])
{
          int i,j,k,o,m;
          INDIVIDUAL tmp;

          for(o=0;o<IND_SIZE-1;o++){
              for(m=IND_SIZE-1;m>0;m--){
                  if(kotai[m-1].E>kotai[m].E){
                      tmp=kotai[m-1];
                      kotai[m-1]=kotai[m];
                      kotai[m]=tmp;
                  }
              }
          }
}

//levy flight
INDIVIDUAL levy_flight(INDIVIDUAL CS_r1,float alpha,float beta)
{
    int i,j,k;
    float u,v,s;
    INDIVIDUAL new_CS;
    alpha=ALPHA2;

    for(j=0;j<J_MAX;j++){
        for(i=0;i<I_MAX;i++){
            u = rand_n()*sigma(beta);
            v = rand_n();
            s = u / pow(fabs(v),1/beta);

new_CS.whid[i][j]=CS_r1.whid[i][j]+alpha*s;
        }
    }
    for(k=0;k<K_MAX;k++){
        for(j=0;j<J_MAX;j++){
            u = rand_n()*sigma(beta);
            v = rand_n();
            s = u / pow(fabs(v),1/beta);

new_CS.wout[j][k]=CS_r1.wout[j][k]+alpha*s;
        }
    }
    return new_CS;
}
//convert CS into NC weights
NEURON_w convert_weight(INDIVIDUAL CS)
{
    int i,j,k;
    NEURON_w NCw;
    for(j=0;j<J_MAX;j++)
             for(i=0;i<I_MAX;i++)
                       NCw.n_ij[i][j]=CS.whid[i][j];
    for(j=0;j<J_MAX;j++){
             for(k=0;k<K_MAX;k++){
                       NCw.n_jk[j][k]=CS.wout[j][k];
             }
    }
    return NCw;
}