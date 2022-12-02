// NC parameters
#define I_MAX 4
#define J_MAX 10
#define K_MAX 1

// NC weights
typedef struct {
            float n_ij[I_MAX][J_MAX];
            float n_jk[J_MAX][K_MAX];
} NEURON_w;

//Input to neuron and Teaching signal
typedef struct {
            float x_i[I_MAX];
            float t_k[K_MAX];
} NEURON_i;

//Output from neuron
typedef struct {
            float net_j[J_MAX],net_k[K_MAX];
            float y_i[I_MAX], y_j[J_MAX], y_k[K_MAX];
} NEURON_o;