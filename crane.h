#define PI  3.1415926

#define STEP 0.01
#define TIME 10.0

// parameters for rotaru crane
#define Grav 9.80665
#define Leng 1.0
#define R   1.0
#define Ht  1.0
#define K   1.0

// inital angle
#define THETA 90.0*PI/180.0
#define ALPHA 0.0*PI/180.0
#define BETA 0.0*PI/180.0

// Reference position and rotary angle
#define Thref 0.0
#define dThref 0.0
#define Xref  R*cos(Thref)
#define Yref  R*sin(Thref)
#define dXref  0.0
#define dYref  0.0

#define Ts 4.0 /** Evaluation start time **/
#define Tm 5.0 /** Evaluation mid  time **/
#define Te 6.0 /** Evaluation end  time **/

#define As1 10.0 /** start slope constant **/
#define Bs1 10.0 /**end  slope constant **/

typedef struct {
            float x1, x2, x3, x4, x5, x6;
}RK;

typedef struct {
            float ft, th;
} IN;