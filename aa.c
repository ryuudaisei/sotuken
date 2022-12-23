BAT bat_algorithm(BAT NC1, BAT NC2, BAT best_NC, int generation, double aveLoud,float Pulse_r1)
{
	int i, j, k;
	BAT new_NC1, new_NC2, tmp_NC;
	
    //最良のコウモリ
	//NC1.freq=(Freq_Max-Freq_Min);
	NC1.freq=(Freq_Max-Freq_Min)*rand()/RAND_MAX;
	for(j=0;j<J;j++) {
		for(i=0;i<I;i++) {
			NC1.vji[j][i]+=(best_NC.pji[j][i]-NC1.pji[j][i])*NC1.freq;
			NC1.pji[j][i]+=NC1.vji[j][i];
			//NC1.freq=*(best_NC.freq);
			
		}
	}
	for(k=0;k<K;k++) {
		for(j=0;j<J;j++) {
			NC1.vkj[k][j]+=(best_NC.pkj[k][j]-NC1.pkj[k][j])*NC1.freq;
			NC1.pkj[k][j]+=NC1.vkj[k][j];
			//NC1.freq=(best_NC.freq);
		}
	}
	NC1=control_simulation(NC1);
	if(isnan(NC1.E)) {
		NC1.E=DBL_MAX;
	}
	
    //良いコウモリ
	//if(0.01> NC1.pulse) {
	if(rand()/RAND_MAX > NC1.pulse) {		
		new_NC1=init_NC1();
		new_NC1.loudness=Loud_0*pow(Loud_r,0.1*(double)generation);	
		new_NC1.pulse=Pulse_0*(1.0-exp(-Pulse_r*(double)Pulse_r1*(double)generation));
		for(j=0;j<J;j++) {
			for(i=0;i<I;i++) {
				new_NC1.pji[j][i]=NC2.pji[j][i]+(rand()/(RAND_MAX/2.0)-1)*aveLoud;
				new_NC1.vji[j][i]=NC2.vji[j][i];
				//new_NC1.freq=NC2.freq;
				//new_NC1.pulse=NC2.pulse;
				//new_NC1.loudness=(NC2.loudness);
			}
		}
		for(k=0;k<K;k++) {
			for(j=0;j<J;j++) {
				new_NC1.pkj[k][j]=NC2.pkj[k][j]+(rand()/(RAND_MAX/2.0)-1)*aveLoud;
				new_NC1.vji[k][j]=NC2.vji[k][j];
				//new_NC1.freq=NC2.freq;
				//new_NC1.pulse=NC2.pulse;
				//new_NC1.loudness=(NC2.loudness);
			}
		}
		new_NC1=control_simulation(new_NC1);
		if(isnan(new_NC1.E)) {
			new_NC1.E=DBL_MAX;
		}
	} else {
		new_NC1.E=DBL_MAX;
		new_NC1.Ey=DBL_MAX;
		new_NC1.Et=DBL_MAX;
	}
	new_NC2=init_NC1();
	new_NC2.loudness=Loud_0*pow(Loud_r,0.1*(double)generation);	
	new_NC2.pulse=Pulse_0*(1.0-exp(-Pulse_r*Pulse_r1*(double)generation));
	new_NC2=control_simulation(new_NC2);
	if(isnan(new_NC2.E)) {
		new_NC2.E=DBL_MAX;
	}
	
    //パルス率を更新
	if(((new_NC1.E<NC1.E) || (new_NC2.E<NC1.E)) && ((rand()/RAND_MAX)<NC1.loudness)) {
			if(new_NC1.E<new_NC2.E) {
				tmp_NC=NC1;
			NC1=new_NC1;
			new_NC1=tmp_NC;
		} else {
			tmp_NC=NC1;
			NC1=new_NC2;
			new_NC2=tmp_NC;
		}
		//NC1.pulse=Pulse_0*(1.0-exp(-Pulse_r*(double)generation));
		//NC1.loudness*=Loud_r;
	}
	return NC1;
}
