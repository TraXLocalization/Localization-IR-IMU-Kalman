/* A simplified one dimensional Kalman filter implementation - actually a single variable low pass filter ;-)
Based on: http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/
*/

#include <math.h>

#ifndef _KalmanSensorFusionn_h
#define _KalmanSensorFusion_h
//#define PI 3.14159
#define N 5

class KalmanSensorFusion {
private:
	/* Kalman filter variables */
	float qx, qy; //process noise covariance

	float xy[2]; //value

	float px, py;//estimation error covariance

	float kx[N], ky[N], k1, k2; //kalman gain

	float meanx, variancex, pdfx, normpdfx[N], sumx, tempx;  // mean & std dev of each measurement
	float meany, variancey, pdfy, normpdfy[N], sumy, tempy;  // mean & std dev of each measurement

	int i, errorSumx, errorSumy, errorPDFx, errorPDFy, errorIMU, errorIR;
	float sumIMU, sumIR;

public:
	KalmanSensorFusion(float process_noise, float estimated_error, float init_x, float init_y) {
		/* The variables are x for the filtered value, q for the process noise,
		r for the sensor noise, p for the estimated error and k for the Kalman Gain.
		The state of the filter is defined by the values of these variables.

		The initial values for p is not very important since it is adjusted
		during the process. It must be just high enough to narrow down.
		The initial value for the readout is also not very important, since
		it is updated during the process.
		But tweaking the values for the process noise and sensor noise
		is essential to get clear readouts.

		For large noise reduction, you can try to start from: (see http://interactive-matter.eu/blog/2009/12/18/filtering-sensor-data-with-a-kalman-filter/ )
		q = 0.125
		r = 32
		p = 1023 //"large enough to narrow down"
		e.g.
		myVar = Kalman(0.125,32,1023,0);
		*/
		this->qx = process_noise;
		this->qy = process_noise;
		this->px = estimated_error;
		this->py = estimated_error;
		this->xy[0] = init_x;
		this->xy[1] = init_y; 
	}

	float* getSigmaFiltered_IMU(float *mx, float *my, float *r1) {

		//zero variables to use
		this->sumx = 0.0, this->tempx = 0.0;
		this->sumy = 0.0, this->tempy = 0.0;

		this->px = this->px + this->qx;
		this->py = this->py + this->qy;

		/***/////////////////////////////////////////////////////////////////////// 
		/***///////////////////////////////////////////////////////////////////////
		
		// INITIAL MEAN
		for(i=0; i<N; i++) {
        	this->sumx += mx[i];
        	this->sumy += my[i];
        	if(sumx == 0.0) errorSumx = 1;		// flag error if the sumx is all zeros.
        	if(sumy == 0.0) errorSumy = 1;		// flag error if the sumy is all zeros.
		}
		this->meanx = this->sumx/N;
		this->meany = this->sumy/N;
		sumx = 0.0; sumy = 0.0;

		if(!errorSumx) {
			// VARIANCE, PDF
			for(i=0; i<N; i++) {
	        	this->variancex += pow(mx[i] - this->meanx, 2);
			}
	    	this->pdfx = sqrt(this->variancex/N);
	    	if(pdfx == 0.0) errorPDFx = 1; 

	    	if(!errorPDFx) {
		    	// NEW SUM, KALMAN GAINS
		    	for(i=0; i<N; i++) {
		        	this->normpdfx[i] = exp(-0.5 * pow( ((mx[i] - this->meanx)/this->pdfx),2) ) / (sqrt(2*PI) * this->pdfx);

		        	this->kx[i] = (this->px) / (this->px + (r1[i]/this->normpdfx[i]));			//kalman gains calculated based on power of sigma points
		    	}

		    	for(i=0; i<N; i++){
					this->xy[0] += this->kx[i]*(mx[i] - this->xy[0]);						//new estimate, based on all sigma points
		    	}

				for(i=0; i<N; i++){
					this->tempx += (1 - this->kx[i]);
				}
				this->px = (this->tempx/N) * this->px;								//new process error, based on power of all sigma points
			}
		}

		if(!errorSumy) {
			// VARIANCE, PDF
			for(i=0; i<N; i++) {
	        	this->variancey += pow(my[i] - this->meany, 2);
			}
	    	this->pdfy = sqrt(this->variancey/N);
	    	if(pdfy == 0.0) errorPDFy = 1;

	    	if(!errorPDFy) {
		    	// NEW SUM, KALMAN GAINS
		    	for(i=0; i<N; i++) {
		        	this->normpdfy[i] = exp(-0.5 * pow( ((my[i] - this->meany)/this->pdfy),2) ) / (sqrt(2*PI) * this->pdfy);

		        	//kalman gains calculated based on power of sigma points
		        	this->ky[i] = (this->py) / (this->py + (r1[i]/this->normpdfy[i]));
		    	}

		    	for(i=0; i<N; i++){
					//new estimate, based on all sigma points
					this->xy[1] += this->ky[i]*(my[i] - this->xy[1]);
		    	}

				for(i=0; i<N; i++){
					
					this->tempy += (1 - this->ky[i]);
				}

				//new process error, based on power of all sigma points
				this->py = (this->tempy/N) * this->py;
			}
		}
		
		// HANDLE ALL ERROR FLAGS
		if(errorSumx) { 
			this->xy[0] = 0.0; 
			//Serial.println("Sum X = 0 ERROR ");
			errorSumx = 0;
		} else if (errorPDFx){
			//Serial.println("PDF X = 0 ERROR ");
			errorPDFx = 0;
		}
		if(errorSumy) { 
			this->xy[1] = 0.0;
			//Serial.println("Sum Y = 0 ERROR ");
			errorSumy = 0;
		} else if (errorPDFy){
			//Serial.println("PDF Y = 0 ERROR ");
			errorPDFy = 0;
		}

		return this->xy;
	}

	float* getLinearFiltered_IR(float *IR, float r2, float sumAccelx, float sumAccely) {
		// Updates and gets the current measurement value 
		//prediction update
		//omit x = x
		this->px = this->px + this->qx;
		this->py = this->py + this->qy;

		this->k2 = this->py / (this->py + r2); 
		this->sumIR = 0.0;

		if(sumAccelx > 0.0)
		this->xy[0] = this->xy[0] + this->k2*(IR[0] - this->xy[0]);
		if(sumAccely > 0.0)
		this->xy[1] = this->xy[1] + this->k2*(IR[1] - this->xy[1]);
		
		this->px = ((1 - this->k2)/2) * this->px;
		this->py = ((1 - this->k2)/2) * this->py;
		

		return this->xy;
	}
	

	void setPosition(float *POS){
		xy[0] = POS[0];
		xy[1] = POS[1];
	}

};

#endif
