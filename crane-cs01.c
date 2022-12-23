/* ########################################
   ### Rotary crane system 
   ### NC optimized by Cuckoo Search(bat_algorithm) method.
   ### Evolution simulation in 1 trial.
   ### Compile: gcc -O2 -o crane-bat_algorithm01 crane-bat_algorithm01.c myfopen.c 
   crane_3dmodel.C neural_network.c cuckoo_search.c crane_control.c -lm
   ### programmed by nakazono. [2021/09/17]
   ######################################## */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include <time.h>

#include "crane.h"
#include "nn.h"
#include "bat.h"


int main(void)
{
	//*ファイルポインタの宣言*//
	FILE *fp3;

	BAT NC[P], best_NC;
	double aveLoud;
	
	int i, j, k, r;
	int s, generation, p,z,BSD;

	float Pulse_r1;
	int SEED, bestseed;
	float best;
	
	if((fp3=fopen("BSD-BAT.dat","w"))==NULL) {
		printf("Can't open file¥n");
		exit(1);
	}
	
	for(s=0;s<1;s++) {
		//printf("%d\n", s);
		//fprintf(fp1, "%d\n", s);

		//seed値ごとの最適化(1000世代まで)を100回
		for(z=100;z<=100;z+=100) {
			BSD=z;
			printf("Seed:%d\n", z);
			fprintf(fp3,"Seed:%d\n",z);
			srand(BSD);
	
		
			for(p=0;p<P;p++) {
				NC[p]=init_NC();
				NC[p]=control_simulation(NC[p]);
				if(isnan(NC[p].E)) {
					NC[p].E=DBL_MAX;;
				}
			}
			sort(NC);
			best_NC=NC[0];
			//printf("E=%lf¥n", best_NC.E);
			//*-----世代数の繰り返し-----*//
			for(generation=0;generation<=G;generation++) {
				//if(generation<=500){
				//	Pulse_r1=0.9;
				//} else if(generation<=1000){
				//	Pulse_r1=0.7;
				//} else if(generation<=1500){
				//	Pulse_r1=0.5;
				//} else if(generation<=2000){
				//	Pulse_r1=0.3;
				//} else if(generation>2000){
				//	Pulse_r1=0.1;
				//}
				aveLoud=0.0;
				Pulse_r1=0.9;
				//*-----個体数の繰り返し-----*//
				for(p=0;p<P;p++) {
					aveLoud+=NC[p].loudness;
				}
				aveLoud/=P;
				for(p=0;p<P;p++){
					//r=(int)(P*Best_rate);
					r=rand()%(int)(P*Best_rate);
					NC[p]=bat_algorithm(NC[p], NC[r], best_NC,generation,aveLoud,Pulse_r1);
					//NC[p].pulse=Pulse_0*(1.0-exp(-Pulse_r*Pulse_r1*(double)generation));
					//NC[p].loudness=Loud_0*pow(Loud_r,0.1*(double)generation);
				}
				sort(NC);

				if(((best_NC.loudness==NC[0].loudness) || (best_NC.pulse==NC[0].pulse)) && (best_NC.E>NC[0].E)) {			
					//if(best_NC.E>NC[0].E) {
					best_NC.pji[J][I]=NC[0].pji[J][I];
					best_NC.pkj[A][J]=NC[0].pkj[A][J];
					best_NC.vji[J][I]=NC[0].vji[J][I];
					best_NC.vkj[A][J]=NC[0].vkj[A][J];			
					best_NC.E=NC[0].E;
					best_NC.Et=NC[0].Et;
					best_NC.Ey=NC[0].Ey;
					best_NC.freq=NC[0].freq;					
					best_NC.loudness=NC[0].loudness;
					//best_NC.pulse=Pulse_0*(1.0-exp(-Pulse_r*(double)generation));
					//best_NC.pulse=NC[0].pulse;
					//}
				}else if(best_NC.E>NC[0].E){			
					//}else(((best_NC.loudness!=NC[0].loudness) || (best_NC.pulse!=NC[0].pulse)) && (best_NC.E>NC[0].E)){
					best_NC=NC[0];
				}
				if(generation==3000) {
					printf("generation: %d %1f\n", generation, best_NC.Ey+best_NC.Et);
					fprintf(fp3,"%1f\n",best_NC.Ey+best_NC.Et);
				}
				printf("%d: E=%lf Ey=%lf Et=%lf\n", generation, best_NC.Ey+best_NC.Et, best_NC.Ey, best_NC.Et);
			}
		}
		for(j=0;j<J;j++) {
			for(i=0;i<I;i++) {
				//fprintf(fp2,"%lf\n", best_NC.pji[j][i]);
			}
		}
		for(k=0;k<K;k++) {
			for(j=0;j<J;j++) {
				//fprintf(fp2,"%lf\n", best_NC.pkj[k][j]);
			}
		}
	}
	fclose(fp3);
	return 0;
	}

	BAT control_simulation(BAT NC)
{
	double t;
	double Xr;
	double Vx;
	double Yr;
	double Vy;
	double Thetar;
	double phi;
	double WT;
	double t1, t2, t3, t4, t5, t6, t7, t8, t9;
	double a11, a12, a13, a14, a21, a22, a23, a24;
	double a31, a32, a33, a34, a41, a42, a43, a44;
	double y_1, y_2, y_3, y_4;
	
	double inp[I], hid[J], out[A];
	double jnet, knet;
	
	double delta;
	
	int i,j,k;

	//*-----初期化-----*//
	Xr=0.0; Vx=V0; Yr=0.0; Vy=0.0;Thetar=0.0;
	delta=0.0; phi=0.0; WT=0.0;
	NC.E=0.0; NC.Ey=0.0; NC.Et=0.0;
				
	y_1 = y_2 = y_3 = y_4 = 0.0;
				
	t1=30.0/V0;
	t2=(30.0+(5.0*Math_PI*12.5))/V0;
	t3=(30.0+(5.0*Math_PI*12.5)+sqrt(64.4))/V0;
	t4=(30.0+2.0*(5.0*Math_PI*12.5)+sqrt(64.4))/V0;
	t5=(30.0+2.0*(5.0*Math_PI*12.5)+sqrt(64.4)+10.0)/V0;
	t6=(30.0+3.0*(5.0*Math_PI*12.5)+sqrt(64.4)+10.0)/V0;
	t7=(30.0+3.0*(5.0*Math_PI*12.5)+2.0*sqrt(64.4)+10.0)/V0;
	t8=(30.0+4.0*(5.0*Math_PI*12.5)+2.0*sqrt(64.4)+10.0)/V0;
	t9=(30.0+4.0*(5.0*Math_PI*12.5)+2.0*sqrt(64.4)+10.0+40.0)/V0;
				
	//*-----計測開始-----*//
	for(t=0.0;t<=TIME;t+=h) {
		//*条件*//
		if(t>=0.0 && t<=t1) {
			//*時間*//
			Xr=V0*t;
			//*車両位置のX座標*//
			Vx=V0;
			//*速度のX成分*//
			Yr=0.0;
			//*車両位置のY座標*//
			Vy=0.0;
			//*速度のY成分*//
			phi=0.0;
			//*角度*//
			Thetar=0.0;
			//*角度(deg)*//
			WT=0.0;
			//*角度の時間変化*//
		}
		else if(t>t1 && t<=t2) {
			for(Thetar=0; Thetar>=-12.5; Thetar-=1.0) {
				phi=Math_PI*Thetar;
			}
			WT=phi/(t2-t1)*(t-t1);
			Xr=R*cos(WT+Math_PI*90)+30.0;
			Vx=-R*(phi/(t2-t1))*sin(WT+Math_PI*90);
			Yr=R*sin(WT+Math_PI*90)-5.0;
			Vy=R*(phi/(t2-t1))*cos(WT+Math_PI*90);
		}
		else if(t>t2 && t<=t3) {
			Thetar=atan2(-1.76, 7.83);
			phi=Thetar;
			WT=phi;
			Xr=7.83/(t3-t2)*(t-t2)+31.08;
			Vx=V0*cos(WT);
			Yr=-1.76/(t3-t2)*(t-t2)-0.12;
			Vy=V0*sin(WT);
		}
		else if(t>t3 && t<=t4) {
			for(Thetar=0; Thetar<=12.5; Thetar+=1.0) {
				phi=Math_PI*Thetar;
			}
			WT=phi/(t4-t3)*(t-t3)+atan2(-1.76, 7.83);
			Xr=R*cos((WT-atan2(-1.76, 7.83))+Math_PI*257.5)+40.0;
			Vx=-R*(phi/(t4-t3))*sin((WT-atan2(-1.76, 7.83))+Math_PI*257.5);
			Yr=R*sin((WT-atan2(-1.76, 7.83))+Math_PI*257.5)+3.0;
			Vy=R*(phi/(t4-t3))*cos((WT-atan2(-1.76, 7.83))+Math_PI*257.5);
		}
		else if(t>t4 && t<=t5) {
			Xr=10.0/(t5-t4)*(t-t4)+40.0;
			Vx=V0;
			Yr=-2.0;
			Vy=0.0;
			phi=0.0;
			Thetar=0.0;
			WT=0.0;
		}
		else if(t>t5 && t<=t6) {
			for(Thetar=0.0; Thetar<=12.5; Thetar+=1.0) {
				phi=Math_PI*Thetar;
			}
			WT=phi/(t6-t5)*(t-t5);
			Xr=R*cos(WT+Math_PI*270)+50.0;
			Vx=-R*(phi/(t6-t5))*sin(WT+Math_PI*270);
			Yr=R*sin(WT+Math_PI*270)+3.0;
			Vy=R*(phi/(t6-t5))*cos(WT+Math_PI*270);
		}
		else if(t>t6 && t<=t7) {
			Thetar=atan2(1.76, 7.83);
			phi=Thetar;
			WT=phi;
			Xr=7.83/(t7-t6)*(t-t6)+51.08;
			Vx=V0*cos(phi);
			Yr=1.76/(t7-t6)*(t-t6)-1.88;
			Vy=V0*sin(phi);
		}
		else if(t>t7 && t<=t8) {
			for(Thetar=0.0; Thetar>=-12.5; Thetar-=1.0) {
				phi=Math_PI*Thetar;
			}
			WT=phi/(t8-t7)*(t-t7)+atan2(1.76, 7.83);
			Xr=R*cos((WT-atan2(1.76, 7.83))+Math_PI*102.5)+60.0;
			Vx=-R*sin(phi/(t8-t7))*sin((WT-atan2(1.76, 7.83))+Math_PI*102.5);
			Yr=R*sin((WT-atan2(1.76, 7.83))+Math_PI*102.5)-5.0;
			Vy=R*(phi/(t8-t7))*cos((WT-atan2(1.76, 7.83))+Math_PI*102.5);
		}
		else {
			Xr=V0*(t-t8)+60.0;
			Vx=V0;
			Yr=0.0;
			Vy=0.0;
			phi=0.0;
			Thetar=0.0;
			WT=0.0;
		}
				
		//*入力層*//
		inp[0]=Yr - y_1;
		inp[1]=Vy - y_2;
		inp[2]=WT - y_3;
				
		for(j=0;j<J;j++) {
			jnet=0.0;
			for(i=0;i<I;i++) {
				jnet+=NC.pji[j][i]*inp[i];
				//前の層からの出力に、重みをかけたものを束ねている//
			}
			hid[j]=SIG(jnet);
			//中間層からの出力//
		}
				
		for(k=0;k<K;k++) {
			knet=0.0;
			for(j=0;j<J;j++) {
				knet+=NC.pkj[k][j] *hid[j];
				//前の層からの出力に、重みをかけたものを束ねている//
			}
			out[k]=SIG(knet);
			//出力層からの出力//
			delta=out[k];
		}
				
		//*ルンゲ=クッタ*//
		// a11=h*f1(y_1, y_2, y_3, y_4, delta);
		// a12=h*f2(y_1, y_2, y_3, y_4, delta);
		// a13=h*f3(y_1, y_2, y_3, y_4, delta);
		// a14=h*f4(y_1, y_2, y_3, y_4, delta);
				
		// a21=h*f1(y_1+a11/2.0, y_2+a12/2.0, y_3+a13/2.0, y_4+a14, delta);
		// a22=h*f2(y_1+a11/2.0, y_2+a12/2.0, y_3+a13/2.0, y_4+a14, delta);
		// a23=h*f3(y_1+a11/2.0, y_2+a12/2.0, y_3+a13/2.0, y_4+a14, delta);
		// a24=h*f4(y_1+a11/2.0, y_2+a12/2.0, y_3+a13/2.0, y_4+a14, delta);
				
		// a31=h*f1(y_1+a21/2.0, y_2+a22/2.0, y_3+a23/2.0, y_4+a24, delta);
		// a32=h*f2(y_1+a21/2.0, y_2+a22/2.0, y_3+a23/2.0, y_4+a24, delta);
		// a33=h*f3(y_1+a21/2.0, y_2+a22/2.0, y_3+a23/2.0, y_4+a24, delta);
		// a34=h*f4(y_1+a21/2.0, y_2+a22/2.0, y_3+a23/2.0, y_4+a24, delta);
				
		// a41=h*f1(y_1+a31, y_2+a32, y_3+a33, y_4+a34, delta);
		// a42=h*f2(y_1+a31, y_2+a32, y_3+a33, y_4+a34, delta);
		// a43=h*f3(y_1+a31, y_2+a32, y_3+a33, y_4+a34, delta);
		// a44=h*f4(y_1+a31, y_2+a32, y_3+a33, y_4+a34, delta);
				
		// y_1+=(a11+2.0*a21+2.0*a31+a41)/6.0;
		// y_2+=(a12+2.0*a22+2.0*a32+a42)/6.0;
		// y_3+=(a13+2.0*a23+2.0*a33+a43)/6.0;
		// y_4+=(a14+2.0*a24+2.0*a34+a44)/6.0;
				
		//評価関数//
		NC.Ey += pow((Yr-y_1), 2);
		//*位置偏差の式*//
		NC.Et += pow((WT-y_3), 2);
		//*角度偏差の式*//
		//printf("%f\n", pow((Yr-y_1), 2));
	}
	//NC.E=NC.Ey+0.01*NC.Et;
	NC.E=NC.Ey+NC.Et;
	//NC.E=NC.Et;
	//NC.E=NC.Ey;
	return NC;
}
float SIG(double u)
{
	return((1.0-exp(-u))/(1.0+exp(-u)));
}

BAT init_NC(void)
{
	int i,j,k;
	BAT init_NC1;
	for(j=0;j<J;j++) {
		for(i=0;i<I;i++) {
			init_NC1.pji[j][i]=2.0*((double)rand()/RAND_MAX)-1.0;
			init_NC1.vji[j][i]=0.0;
		}
	}
	for(k=0;k<K;k++) {
		for(j=0;j<J;j++) {
			init_NC1.pkj[A][j]=2.0*((double)rand()/RAND_MAX)-1.0;
			init_NC1.vkj[A][j]=0.0;
		}
	}
	init_NC1.freq=0.0;
	init_NC1.pulse=0.0;
	init_NC1.loudness=Loud_0;
	init_NC1.E=0.0;
	init_NC1.Ey=0.0;
	init_NC1.Et=0.0;
	
	return init_NC1;
}

BAT init_NC1(void)
{
	int i,j,k,l;
	BAT init_NC1;
	for(j=0;j<J;j++) {
		for(i=0;i<I;i++) {
			init_NC1.pji[j][i]=2.0*((double)rand()/RAND_MAX)-1.0;
			init_NC1.vji[j][i]=0.0;
		}
	}
	for(k=0;k<K;k++) {
		for(j=0;j<J;j++) {
			init_NC1.pkj[A][j]=2.0*((double)rand()/RAND_MAX)-1.0;
			init_NC1.vkj[A][j]=0.0;
		}
	}
	//srand(200);
	init_NC1.freq=0.0;
	init_NC1.pulse=0.0;
	init_NC1.loudness=Loud_0;
	//init_NC1.loudness=1000*(rand()/RAND_MAX);
	//init_NC1.loudness=(rand()/RAND_MAX);
	init_NC1.E=0.0;
	init_NC1.Ey=0.0;
	init_NC1.Et=0.0;
	
	return init_NC1;
}

void sort(BAT NC1[])
{
	int i, j, k, m, n;
	
	BAT tmp;
	
	for(m=0;m<P;m++) {
		for(n=m+1;n<P;n++) {
			if(NC1[m].E>NC1[n].E) {
				tmp=NC1[m];
				NC1[m]=NC1[n];
				NC1[n]=tmp;
			}
		}
	}
}

// // 評価出力
// fprint(fp1,"%12f\n",bat[best].E)