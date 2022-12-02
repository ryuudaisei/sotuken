#include <stdio.h> 
#include <stdlib.h> 
#include <math.h> 

#include "nn.h" 

/** sigmoid function x=[0.0,1.0] **/ 
float sigmoid1(float x) 
{
    return 1.0/(1.0+exp(-x)); 
}

/** sigmoid function x=[-1.0,1.0] **/ 
float sigmoid2(float x) 
{
    return (1.0-exp(-x))/(1.0+exp(-x)); 
}

// three layered neural network controller
NEURON_o neural_network(NEURON_i nin, NEURON_w w) 
{
    NEURON_o nout; 
    int i, j, k;
    for(i=0;i<I_MAX;i++){ 
        nout.y_i[i]=nin.x_i[i]; 
    }
    for(j=0;j<J_MAX;j++){ 
        nout.net_j[j] = 0.0; 
        for(i=0;i<I_MAX;i++){ 
            nout.net_j[j]+=w.n_ij[i][j]*nout.y_i[i];
        }
        nout.y_j[j]=sigmoid2(nout.net_j[j]);
    }
    for(k=0;k<K_MAX;k++){ 
        nout.net_k[k] = 0.0; 
        for(j=0;j<J_MAX;j++){ 
            nout.net_k[k] += w.n_jk[j][k]*nout.y_j[j]; 
        }
        nout.y_k[k]=nout.net_k[k]; 
    }

    return nout;
}

