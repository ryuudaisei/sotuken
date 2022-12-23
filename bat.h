// // Cuckoo parameters 
// #define ITERATION 10000  //iteration
// #define ABA_RATE 0.1      //abandon rate
// #define IND_SIZE 100    //INDIVIDUAL size
// #define ALPHA2 0.1        //step length parameter
// #define BETA2 1.5         //scaling index 
// #define XMAX 1.0

// // individual of cuckoo(NC)　INDIVIDUALという名前の構造体ここに必要な値変数を描く
// //最終的な評価値
// typedef struct{ 
//            float whid[I_MAX][J_MAX]; 
//            float wout[J_MAX][K_MAX]; 
//            //評価
//     float E;
// }INDIVIDUAL;

#define mM 1300
#define IM 1600
#define l_f 1.05
#define l_r 1.48

#define K_f 35000.0
#define K_r 40000.0

#define V0 (36/3.6)

#define h 0.01

#define I 3
#define J 8
#define A 1

#define P 100
//#define Tm 12
#define G 300
//#define RATE 20
#define Math_PI 3.14/180
// #define R 5.0
//*円軌道の半径*//

#define Freq_Min 0.0
#define Freq_Max 1.0
#define Loud_0 1.0
#define Loud_01 1,0
#define Loud_r 0.9

#define Pulse_0 0.5
#define Pulse_r 1.0
#define Best_rate 0.1

typedef struct {
	double E, Ey, Et;			// 評価値
	double pji[J][I],pkj[A][J];		// 評価重み　入力-中間 中間-出力
	double vji[J][I],vkj[A][J];		// 速度　入力-中間 中間-出力
	double freq;				// 周波数
	double pulse;				// パルス率
	double loudness;			// 音量
} BAT;

BAT control_simulation(BAT NC);
BAT init_NC(void);
void sort(BAT NC1[]);
BAT bat_algorithm(BAT NC1, BAT NC2, BAT best_NC, int generation, double aveLoud,float Pulse_r1);
float SIG(double u);

// //*ルンゲクッタによる運動方程式*//
// double f1(double y_1, double y_2, double y_3, double y_4, double delta);
// double f2(double y_1, double y_2, double y_3, double y_4, double delta);
// double f3(double y_1, double y_2, double y_3, double y_4, double delta);
// double f4(double y_1, double y_2, double y_3, double y_4, double delta);
