/*
 * ahrs.cpp
 *
 *  Created on: Jun 12, 2014
 *      Author: ohachett
 *
 *  /home/ohachett/CodeSourcery/bin/arm-none-linux-gnueabi-gcc -march=armv7-a test.cpp -o toto.elf -lm
 *
 */

#include "ahrs.h"


struct param
{
	ahrs* nouveau;
	float dt;
};

pthread_t thread_recup_data, thread_kalman;
pthread_attr_t attr, attr2, *attrp, *attrp2;

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

/*
 * Met à jour les données du magnétomètre, accéléromètre et gyroscope en même temps
 */
void ahrs::Update(float mx, float my, float mz, float ax, float ay, float az, float gp, float gq, float gr){
	ahrs::UpdateMagnetometer(mx,my,mz);
	ahrs::UpdateAccelerometer(ax,ay,az);
	ahrs::UpdateGyrometer(gp,gq,gr);
	ahrs::Update();
}

/*
 * Initialise l'ahrs avec les données accéléromètre et magnétomètre
 */
void ahrs::Initialize(){
	if(started==false){
		if(initialized==false){
			std::FILE * file;
			if(!(file=std::fopen("config.txt","r"))){
				this->Etalonnage();
				file=std::fopen("config.txt","r");
			}
			fscanf(file,"%f\n",&gravity);
			for(int i=0;i<3;i++){
				fscanf(file,"%f %f\n",&max_magn[i],&min_magn[i]);
			}
			initialized=true;
		}
		std::vector<float> temp(3);
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				if(i!=j)
					R[i][j]=0;
				else
					R[i][j]=1;
			}
		}
		Qua=ahrs::MatToQua(R);
		Q[0][0]=Q[1][1]=0.0001;
		Q[1][0]=Q[0][1]=0;
		for(int i=0;i<3;i++){
			for(int j=0;j<2;j++){
				for(int k=0;k<2;k++){
					P[i][j][k]=0;
				}
			}
		}
	}
}

/*
 * Spécifie le temps d'échantillonage
 */
void ahrs::Set(float dt){
	if(started==false){
		if(dt<0.02)
			this->dt=0.02;
		else
			this->dt=dt;
	}
}

/*
 * Permet de spécifier le temps déchantillonage, le coefficient intégrateur et proportionnel du correcteur
 */
void ahrs::Set(float dt, float ki, float kp){
	if(started==false){
		this->Set(dt);
		if(ki>=0)
			this->Ki=ki;
		if(kp>=0)
			this->Kp=kp;
	}
}

/*
 * Permet de spécifier le temps déchantillonage, le coefficient intégrateur et proportionnel du correcteur
 * La confiance accordée à l'acceleromètre et magnétomètre et le coefficient de confiance du correcteur total
 */
void ahrs::Set(float dt, float ki, float kp, float ka, float km, float kt){
	if(started==false){
		this->Set(dt,ki,kp);
		if(ka>=0)
			this->Ka=ka;
		if(km>=0)
			this->Km=km;
		if(kt>=0)
			this->Kt=kt;
	}
}

/*
 * Mettre à jomz;//mzur les données du magnétomètre (mx,my,mz)
 */
void ahrs::UpdateMagnetometer(float mx, float my, float mz){
	std::vector<float> temp(3,0.0);
	if(max_magn[0]-min_magn[0]!=0 && max_magn[1]-min_magn[1]!=0 && max_magn[2]-min_magn[2]!=0){
		temp[0]=(mx-min_magn[0]/2)/(max_magn[0]-min_magn[0]);
		temp[1]=(my-min_magn[1]/2)/(max_magn[1]-min_magn[1]);
		temp[2]=(mz-min_magn[2]/2)/(max_magn[2]-min_magn[2]);
	}
	if(!kalman_filter){
		for(int i = 0;i<3;i++){
			magn[i] = magn[i]*3/4 + temp[i]/4;//Low pass
		}
		float norme = pow_sum(magn);
		norme = sqrt(norme);
		if(norme>0){
			for(int i=0;i<3;i++){
				magn[i]/=norme;
			}
		}
	}
	else{
		for(int i = 0;i<3;i++){
			magn[i] = temp[i];
		}
	}
}

/*
 * Mettre à jour les données de l'accéléromètre (ax,ay,az)
 */
void ahrs::UpdateAccelerometer(float ax, float ay, float az){
	std::vector<float> temp(3);
	temp[0]=ax;
	temp[1]=ay;
	temp[2]=az;
	if(!kalman_filter){
		for(int i = 0;i<3;i++){
			accel[i] = accel[i]*3/4 + temp[i]/4;//Low pass
		}
		float norme = pow_sum(accel);
		norme = sqrt(norme);
		if(norme>0){
			for(int i=0;i<3;i++){
				accel[i]/=norme;
			}
		}
	}
	else{
		for(int i = 0;i<3;i++){
			accel[i] = temp[i];
		}
	}
}

/*
 * Mettre à jour les données du gyromètre (gx,gy,gz)
 */
void ahrs::UpdateGyrometer(float gp, float gq, float gr){
	std::vector<float> temp(3);
	temp[0]=gp;
	temp[1]=gq;
	temp[2]=gr;
	if(!kalman_filter){
		for(int i = 0;i<3;i++){
			gyro[i] = gyro[i]*3/4 + temp[i]/4;//Low pass
		}
	}
	else{
		for(int i = 0;i<3;i++){
			gyro[i] = temp[i];
		}
	}
}

/*
 * Lance le calcul du PI / Filtre Kalman et met à jour les sorties
 */
void ahrs::Update(){
	float variation;
	if(kalman_filter){
		gyro=kalman_update(gyro);
	}
	if(quaternion){
		//xioTechnologies/Open-Source-AHRS-With-x-IMU
		//Reference direction of Earth's magnetic field
		float q1 = Qua[0], q2 = Qua[1], q3 = Qua[2], q4 = Qua[3]; // short name local variable for readability
		float norm;
		float hx, hy, bx, bz;
		float vx, vy, vz, wx, wy, wz;
		float ex, ey, ez;
		float pa, pb, pc;
		float ax=accel[0], ay=accel[1] , az=accel[2];
		float mx=magn[0], my=magn[1] , mz=magn[2];

		// Auxiliary variables to avoid repeated arithmetic
		float q1q1 = q1 * q1;
		float q1q2 = q1 * q2;
		float q1q3 = q1 * q3;
		float q1q4 = q1 * q4;
		float q2q2 = q2 * q2;
		float q2q3 = q2 * q3;
		float q2q4 = q2 * q4;
		float q3q3 = q3 * q3;
		float q3q4 = q3 * q4;
		float q4q4 = q4 * q4;

		// Reference direction of Earth's magnetic field
		hx = 2 * mx * (0.5 - q3q3 - q4q4) + 2 * my * (q2q3 - q1q4) + 2 * mz * (q2q4 + q1q3);
		hy = 2 * mx * (q2q3 + q1q4) + 2 * my * (0.5 - q2q2 - q4q4) + 2 * mz * (q3q4 - q1q2);
		bx = sqrt((hx * hx) + (hy * hy));
		bz = 2 * mx * (q2q4 - q1q3) + 2 * my * (q3q4 + q1q2) + 2 * mz * (0.5 - q2q2 - q3q3);

		// Estimated direction of gravity and magnetic field
		vx = 2 * (q2q4 - q1q3);
		vy = 2 * (q1q2 + q3q4);
		vz = q1q1 - q2q2 - q3q3 + q4q4;
		wx = 2 * bx * (0.5 - q3q3 - q4q4) + 2 * bz * (q2q4 - q1q3);
		wy = 2 * bx * (q2q3 - q1q4) + 2 * bz * (q1q2 + q3q4) ;
		wz = 2 * bx * (q1q3 + q2q4) + 2 * bz * (0.5 - q2q2 - q3q3);

		// Error is cross product between estimated direction and measured direction of gravity
		ex = (Ka*(ay * vz - az * vy) + Km*(my * wz - mz * wy))/(Ka+Km);
		ey = (Ka*(az * vx - ax * vz) + Km*(mz * wx - mx * wz))/(Ka+Km);
		ez = (Ka*(ax * vy - ay * vx) + Km*(mx * wy - my * wx))/(Ka+Km);
		corri[0] += ex; // accumulate integral error
		corri[1] += ey;
		corri[2] += ez;

		// Apply feedback terms
		gyro[0] = gyro[0] + Kt*(Kp * ex + Ki*corri[0]*dt);
		gyro[1] = gyro[1] + Kt*(Kp * ey + Ki*corri[1]*dt);
		gyro[2] = gyro[2] + Kt*(Kp * ez + Ki*corri[2]*dt);

		// Integrate rate of change of quaternion
		pa = q2;
		pb = q3;
		pc = q4;
		q1 = q1 + (-q2 * gyro[0] - q3 * gyro[1] - q4 * gyro[2])/2 * dt;
		q2 = pa + (q1 * gyro[0] + pb * gyro[2] - pc * gyro[1])/2 * dt;
		q3 = pb + (q1 * gyro[1] - pa * gyro[2] + pc * gyro[0])/2 * dt;
		q4 = pc + (q1 * gyro[2] + pa * gyro[1] - pb * gyro[0])/2 * dt;

		// Normalise quaternion
		norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
		norm = 1.0f / norm;
		Qua[0] = q1 * norm;
		Qua[1] = q2 * norm;
		Qua[2] = q3 * norm;
		Qua[3] = q4 * norm;
	}
	else{
		//PI (drift adjust)
		variation=0;
		for(int i = 0;i<3;i++){
			variation+=accel[i] * magn[i];
		}
		//Verlet integration
		for(int i = 0;i<3;i++){
			accel[i]=accel[i]-(variation*Km/(Ka+Km))*magn[i];
			magn[i]=magn[i]-(variation*Ka/(Ka+Km))*accel[i];
		}

		std::vector<float> temp1(3);
		temp1=this->cross_product(R[2],accel);

		std::vector<float> temp2(3);
		temp2=this->cross_product(R[0],magn);

		std::vector<float> temp(3);
		temp=cross_product(accel,magn);

		std::vector<float> temp3(3);
		temp3=this->cross_product(R[1],temp);
		for(int i = 0;i<3;i++)
			corr[i]=0;
		for(int i = 0;i<3;i++){
			corr[i]+=temp1[i];
			corr[i]+=temp2[i]/2;
			corr[i]+=temp3[i]/2;
		}
		for(int i = 0;i<3;i++){
			corri[i]=corri[i]+Ki*corr[i]*dt;
			corr[i]=Kt*(Kp*corr[i]+corri[i]);
			gyro[i]-=corr[i];
		}
		//R(t+dt);
		std::vector< std::vector<float> > dTheta(3,std::vector<float>(3));
		dTheta[0][0]=1;
		dTheta[0][1]=-gyro[2]*dt;
		dTheta[0][2]=gyro[1]*dt;
		dTheta[1][0]=gyro[2]*dt;
		dTheta[1][1]=1;
		dTheta[1][2]=-gyro[0]*dt;
		dTheta[2][0]=-gyro[1]*dt;
		dTheta[2][1]=gyro[0]*dt;
		dTheta[2][2]=1;
		R=multiply(R,dTheta);
		//Error Alignement
		variation=0;
		for(int i = 0;i<3;i++){
			variation+=R[0][i] * R[1][i];
		}
		//Verlet integration
		for(int i = 0;i<3;i++){
			R[0][i]=R[0][i]-(variation/2)*R[1][i];
			R[1][i]=R[1][i]-(variation/2)*R[0][i];
		}
		//Z=XxY
		R[2]=this->cross_product(R[0],R[1]);
		//Normalization
		float square[3];
		for(int i=0;i<3;i++){
			square[i]=this->pow_sum(R[i]);
			square[i]=1/sqrt(square[i]);
			for(int j=0;j<3;j++){
				R[i][j]*=square[i];
			}
		}
	}
}

/*
 * Calcul de x²+y²+z²
 */
float ahrs::pow_sum(std::vector<float> V){
	float sum=0;
	for(int i=0;i<3;i++){
		sum+=V[i]*V[i];
	}
	return sum;
}

/*
 * Transforme des données matrice en quaternion
 */
std::vector<float> ahrs::MatToQua(std::vector< std::vector<float> > V){
	std::vector<float> Q(4,0.0);
	float trace = V[0][0] + V[1][1] + V[2][2];
	if( trace > 0 ) {
		float s = 0.5f / sqrtf(trace+ 1.0f);
		Q[0] = 0.25f / s;
		Q[1] = ( V[2][1] - V[1][2] ) * s;
		Q[2] = ( V[0][2] - V[2][0] ) * s;
		Q[3] = ( V[1][0] - V[0][1] ) * s;
	} else {
		if ( V[0][0] > V[1][1] && V[0][0] > V[2][2] ) {
			float s = 2.0f * sqrtf( 1.0f + V[0][0] - V[1][1] - V[2][2]);
			Q[0] = (V[2][1] - V[1][2] ) / s;
			Q[1] = 0.25f * s;
			Q[2] = (V[0][1] + V[1][0] ) / s;
			Q[3] = (V[0][2] + V[2][0] ) / s;
		} else if (V[1][1] > V[2][2]) {
			float s = 2.0f * sqrtf( 1.0f + V[1][1] - V[0][0] - V[2][2]);
			Q[0] = (V[0][2] - V[2][0] ) / s;
			Q[1] = (V[0][1] + V[1][0] ) / s;
			Q[2] = 0.25f * s;
			Q[3] = (V[1][2] + V[2][1] ) / s;
		} else {
			float s = 2.0f * sqrtf( 1.0f + V[2][2] - V[0][0] - V[1][1] );
			Q[0] = (V[1][0] - V[0][1] ) / s;
			Q[1] = (V[0][2] + V[2][0] ) / s;
			Q[2] = (V[1][2] + V[2][1] ) / s;
			Q[3] = 0.25f * s;
		}
	}
	return Q;
}


/*
 * Renvoit le quaternion
 */
std::vector<float> ahrs::GetQuaternion() {
	pthread_mutex_lock(&mutex);
	if(!quaternion){
		Qua = ahrs::MatToQua(R);
	}
	pthread_mutex_unlock(&mutex);
	return Qua;
}

/*
 * Renvoit la matrice de rotation
 */
std::vector< std::vector<float> > ahrs::GetRotationMatrix(){
	pthread_mutex_lock(&mutex);
	if(quaternion){
		float sqx = Qua[1]*Qua[1];
		float sqy = Qua[2]*Qua[2];
		float sqz = Qua[3]*Qua[3];

		R[0][0] = 1 - 2*sqy - 2*sqz ;
		R[1][1] = 1 - 2*sqx - 2*sqz ;
		R[2][2] = 1 - 2*sqx - 2*sqy;

		float tmp1 = Qua[1]*Qua[2];
		float tmp2 = Qua[3]*Qua[0];
		R[1][0] = 2.0 * (tmp1 + tmp2) ;
		R[0][1] = 2.0 * (tmp1 - tmp2) ;

		tmp1 = Qua[1]*Qua[3];
		tmp2 = Qua[2]*Qua[0];
		R[2][0] = 2.0 * (tmp1 - tmp2) ;
		R[0][2] = 2.0 * (tmp1 + tmp2) ;
		tmp1 = Qua[2]*Qua[3];
		tmp2 = Qua[1]*Qua[0];
		R[2][1] = 2.0 * (tmp1 + tmp2) ;
		R[1][2] = 2.0 * (tmp1 - tmp2) ;
	}
	pthread_mutex_unlock(&mutex);
	return R;
}

/*
 * Permet de choisir la confiance accordée au filtre de kalman
 */
void ahrs::SetKalman(float kal){
	if(started==false){
		this->SetKalman(true);
		if(kal>=0)
			this->Kal=kal;
	}
}

/*
 * Permet de spécifier si kalman est utilisé
 */
void ahrs::SetKalman(bool use){
	if(started==false){
		kalman_filter=use;
	}
}

/*
 * Permet de spécifier l'utilisation des quaternions dans les calculs
 */
void ahrs::SetQuaternion(bool use){
	if(started==false){
		quaternion=use;
	}
}

/*
 * Une itération de kalman (Matrice rotation)
 */
std::vector<float> ahrs::kalman_update(std::vector<float> V){
	float Y[3]={0};
	float K[3][2]={{0}};
	float S;
	float R=0.01;
	//Mise à jour
	//K = P*H^*S (S=R+HPH^)
	for(int i=0;i<3;i++){
		S=R+P[i][1][1];
		for(int j=0;j<2;j++)
			K[i][j]=P[i][j][1]/S;
	}
	//P = (I-KH)P
	std::vector< std::vector<float> > Matrice(2,std::vector<float>(2,0.0));
	for(int i=0;i<3;i++){
		Matrice[0][0]=P[i][0][0]-K[i][0]*P[i][1][0];
		Matrice[0][1]=P[i][0][1]-K[0][i]*P[i][1][1];
		Matrice[1][0]=(1-K[i][1])*P[i][1][0];
		Matrice[1][1]=(1-K[i][1])*P[i][1][1];
		for(int j=0;j<2;j++){
			for(int k=0;k<2;k++){
				P[i][j][k]=Matrice[j][k];
			}
		}
	}
	//Y=M-H*W;
	for(int i=0;i<3;i++){
		Y[i]=V[i]-W[i][1];
	}
	//W=W+K*Y
	for(int i=0;i<3;i++){
		W[i][0]+=K[i][0]*Y[i];
		W[i][1]+=K[i][1]*Y[i];
	}

	////////////////////////////
	//Send corrected
	for(int i=0;i<3;i++){
		old[i]=V[i];
		V[i]=V[i]*(1-Kal)+W[i][1]*Kal;
	}
	////////////////////////////

	//Predict
	for(int i=0;i<3;i++){
		W[i][1]+=W[i][0]*dt;
	}

	//P = A*P*A^ + Q
	for(int i=0;i<3;i++){
		Matrice[0][0]=P[0][0][i];
		Matrice[0][1]=P[0][0][i]*dt+P[0][1][i];
		Matrice[1][0]=P[0][0][i]*dt+P[1][0][i];
		Matrice[1][1]=(Matrice[1][0]+P[0][1][i])*dt+P[1][1][i];
		for(int j=0;j<2;j++){
			for(int k=0;k<2;k++){
				P[i][j][k]=Matrice[j][k];
			}
		}
		P[0][0][i]+=Q[0][0];
		P[1][1][i]+=Q[1][1];
	}

	//printR(0,P);
	return V;
}

/*
 * Produit en croix
 */
std::vector<float> ahrs::cross_product(std::vector<float> V1, std::vector<float> V2){
	std::vector<float> retour(3);
	retour[0] = V1[1] * V2[2] - V2[1] * V1[2];
	retour[1] = V1[2] * V2[0] - V2[2] * V1[0];
	retour[2] = V1[0] * V2[1] - V2[0] * V1[1];
	return retour;
}

/*
 * Multiplication de deux matrices (R*dTheta)
 */
std::vector< std::vector<float> > ahrs::multiply(std::vector< std::vector<float> > V1,std::vector< std::vector<float> > V2){
	std::vector< std::vector<float> > Matrice(3,std::vector<float>(3));
	for(int i = 0;i<3;i++){
		for(int j = 0;j<3;j++){
			for(int k = 0;k<3;k++){
				Matrice[i][j] +=  V1[i][k] * V2[k][j];
			}
		}
	}
	return Matrice;
}

/*
 * Retourne les angles d'euler
 */
std::vector<float> ahrs::GetEuler(){
	std::vector<float> retour(3,0.0);
	//Phi,Theta,Psi
	pthread_mutex_lock(&mutex);
	if(!quaternion)
		Qua=this->MatToQua(R);
	float temp=2*Qua[1]*Qua[3]+2*Qua[0]*Qua[2];
	retour[0] = atan2(2*Qua[2]*Qua[3]-2*Qua[0]*Qua[1],2*Qua[0]*Qua[0]+2*Qua[3]*Qua[3]-1);
	retour[1] = -atan(temp/sqrt(1-temp*temp));
	retour[2] = atan2(2*Qua[1]*Qua[2]-2*Qua[0]*Qua[3],2*Qua[0]*Qua[0]+2*Qua[1]*Qua[1]-1);
	for(int i=0;i<3;i++){
		retour[i]*=360/(2*M_PI);
	}
	pthread_mutex_unlock(&mutex);
	return retour;
}

/*
 * Routine d'étalonnage, suivre les indications à l'écran
 */
void ahrs::Etalonnage(){
	std::FILE * file;
	float ax,ay,az,mx,my,mz,gx,gy,gz;
	Navdata::init();
	file=std::fopen("config.txt","w");
	ahrs etalon;
	etalon.SetQuaternion(true);
	int etalonnage=0;
	mx = AHRS_HPP_MX * Navdata::IMU::Magnetometer::getY();
	my = AHRS_HPP_MY * Navdata::IMU::Magnetometer::getX();
	mz = AHRS_HPP_MZ * Navdata::IMU::Magnetometer::getZ();
	etalon.max_magn[0]=etalon.min_magn[0]=mx;
	etalon.max_magn[1]=etalon.min_magn[1]=my;
	etalon.max_magn[2]=etalon.min_magn[2]=mz;
	float angle_min=0;
	float angle_max=0;
	float avancement=0;
	int passage=0;
	printf("Effectuez un à deux tours avec le drone placé à l'horizontale\n");
	for(int i=0;i<40;i++){
		printf("-");
	}
	printf("\r");fflush(stdout);
	etalon.gravity=0;
	float iterations=0;
	while(etalonnage<5000){
		for(int i=0;i<5;i++){
			usleep(2000);
			Navdata::update();
			Navdata::IMU::update();
			gx = Navdata::IMU::Gyroscope::getX();
			gy = Navdata::IMU::Gyroscope::getY();
			gz = Navdata::IMU::Gyroscope::getZ();

			ax = AHRS_HPP_AX * Navdata::IMU::Accelerometer::getX();
			ay = AHRS_HPP_AY * Navdata::IMU::Accelerometer::getY();
			az = AHRS_HPP_AZ * Navdata::IMU::Accelerometer::getZ();

			mx = AHRS_HPP_MX * Navdata::IMU::Magnetometer::getY();
			my = AHRS_HPP_MY * Navdata::IMU::Magnetometer::getX();
			mz = AHRS_HPP_MZ * Navdata::IMU::Magnetometer::getZ();

			etalon.UpdateAccelerometer(ax,ay,az);
			etalon.UpdateMagnetometer(mx,my,mz);
			etalon.UpdateGyrometer(gx,gy,gz);
			if(mx>etalon.max_magn[0] || etalon.max_magn[0]==0)
				etalon.max_magn[0]=mx;
			else if(mx<etalon.min_magn[0] || etalon.min_magn[0]==0)
				min_magn[0]=mx;
			if(my>etalon.max_magn[1] || etalon.max_magn[1]==0)
				etalon.max_magn[1]=my;
			else if(my<etalon.min_magn[1] || etalon.min_magn[1]==0)
				etalon.min_magn[1]=my;
			if(mz>etalon.max_magn[2] || etalon.max_magn[2]==0)
				etalon.max_magn[2]=mz;
			else if(mz<etalon.min_magn[2] || etalon.min_magn[2]==0)
				etalon.min_magn[2]=mz;
			etalon.gravity+=az;
			iterations++;
		}
		if(pow_sum(etalon.magn)!=0){
			etalon.Update();
			float temp=etalon.GetQuaternion()[0];
			if(temp>angle_max)
				angle_max=temp;
			else if(temp<angle_min)
				angle_min=temp;
			if(angle_max-angle_min>1.95){
				etalonnage=passage*2500;
				angle_max=0;
				angle_min=0;
				avancement=0;
				passage++;
			}
			if(avancement+5<(angle_max-angle_min)*10){
				for(int i=0;i<(angle_max-angle_min)*10+passage*10;i++)
					printf("|");fflush(stdout);
				printf("\r");
				avancement=(angle_max-angle_min)/10;
			}
			etalonnage++;
		}
	}
	etalon.gravity/=iterations;
	fprintf(file,"%f\n",etalon.gravity);
	printf("\n");
	printf("Effectuez un à deux tours avec le drone placé à la verticale\n");
	angle_min=0;
	angle_max=0;
	avancement=0;
	etalonnage=0;
	passage=0;
	for(int i=0;i<40;i++){
		printf("-");
	}
	printf("\r");fflush(stdout);
	while(etalonnage<5000){
		for(int i=0;i<5;i++){
			usleep(2000);
			Navdata::update () ;
			Navdata::IMU::update () ;
			gx = Navdata::IMU::Gyroscope::getX();
			gy = Navdata::IMU::Gyroscope::getY();
			gz = Navdata::IMU::Gyroscope::getZ();

			ax = AHRS_HPP_AX * Navdata::IMU::Accelerometer::getX();
			ay = AHRS_HPP_AY * Navdata::IMU::Accelerometer::getY();
			az = AHRS_HPP_AZ * Navdata::IMU::Accelerometer::getZ();

			mx = AHRS_HPP_MX * Navdata::IMU::Magnetometer::getY();
			my = AHRS_HPP_MY * Navdata::IMU::Magnetometer::getX();
			mz = AHRS_HPP_MZ * Navdata::IMU::Magnetometer::getZ();

			etalon.UpdateAccelerometer(ax,ay,az);
			etalon.UpdateMagnetometer(mx,my,mz);
			etalon.UpdateGyrometer(gx,gy,gz);
			if(mx>etalon.max_magn[0])
				etalon.max_magn[0]=mx;
			else if(mx<etalon.min_magn[0])
				etalon.min_magn[0]=mx;
			if(my>etalon.max_magn[1])
				etalon.max_magn[1]=my;
			else if(my<etalon.min_magn[1])
				etalon.min_magn[1]=my;
			if(mz>etalon.max_magn[2])
				etalon.max_magn[2]=mz;
			else if(mz<etalon.min_magn[2])
				etalon.min_magn[2]=mz;
		}
		if(pow_sum(etalon.magn)!=0){
			etalon.Update();
			float temp=etalon.GetRotationMatrix()[2][1];
			if(temp>angle_max)
				angle_max=temp;
			else if(temp<angle_min)
				angle_min=temp;
			if(angle_max-angle_min>1.95){
				etalonnage=passage*2500;
				angle_max=0;
				angle_min=0;
				avancement=0;
				passage++;
			}
			if(avancement+5<(angle_max-angle_min)*10){
				for(int i=0;i<(angle_max-angle_min)*10+passage*10;i++)
					printf("|");fflush(stdout);
				printf("\r");
				avancement=(angle_max-angle_min)/10;
			}
			etalonnage++;
		}
	}
	//CONFIG.TXT
	for(int i=0;i<3;i++){
		fprintf(file,"%f %f\n",etalon.max_magn[i],etalon.min_magn[i]);
	}
	fclose(file);
	printf("\n");
	printf("Etalonnage terminé\n");
}

struct periodic_info
{
	int timer_fd;
	unsigned long long wakeups_missed;
};

static int make_periodic (unsigned int period, struct periodic_info *info)
{
	int ret;
	unsigned int ns;
	unsigned int sec;
	int fd;
	struct itimerspec itval;

	/* Create the timer */
	fd = timerfd_create (CLOCK_MONOTONIC, 0);
	info->wakeups_missed = 0;
	info->timer_fd = fd;
	if (fd == -1)
		return fd;

	/* Make the timer periodic */
	sec = period/1000000;
	ns = (period - (sec * 1000000)) * 1000;
	itval.it_interval.tv_sec = sec;
	itval.it_interval.tv_nsec = ns;
	itval.it_value.tv_sec = sec;
	itval.it_value.tv_nsec = ns;
	ret = timerfd_settime (fd, 0, &itval, NULL);
	return ret;
}


static void wait_period (struct periodic_info *info)
{
	unsigned long long missed;
	int ret;

	/* Wait for the next timer event. If we have missed any the
	   number is written to "missed" */
	ret = read (info->timer_fd, &missed, sizeof (missed));
	if (ret == -1)
	{
		perror ("read timer");
		return;
	}

	/* "missed" should always be >= 1, but just to be sure, check it is not 0 anyway */
	if (missed > 0)
		info->wakeups_missed += (missed - 1);
}	


//Tache du thread recuperation data
static void *thread_recup(void *parametres)
{	
	struct param *pp = (struct param *)parametres;
	struct periodic_info info;
	float gx = 0, gy = 0, gz = 0 ;
	float mx = 0, my = 0, mz = 0 ;
	float ax = 0, ay = 0, az = 0 ;

	int j=0;
	int k=0;
	int i=0;
	make_periodic (2000, &info);
	while (1)
	{
		Navdata::update () ;
		Navdata::IMU::update () ;

		gx = Navdata::IMU::Gyroscope::getX();
		gy = Navdata::IMU::Gyroscope::getY();
		gz = Navdata::IMU::Gyroscope::getZ();

		ax = AHRS_HPP_AX * Navdata::IMU::Accelerometer::getX();
		ay = AHRS_HPP_AY * Navdata::IMU::Accelerometer::getY();
		az = AHRS_HPP_AZ * Navdata::IMU::Accelerometer::getZ();

		mx = AHRS_HPP_MX * Navdata::IMU::Magnetometer::getY();
		my = AHRS_HPP_MY * Navdata::IMU::Magnetometer::getX();
		mz = AHRS_HPP_MZ * Navdata::IMU::Magnetometer::getZ();
		pthread_mutex_lock(&mutex);
		pp->nouveau->UpdateAccelerometer(ax,ay,az);
		pp->nouveau->UpdateMagnetometer(mx,my,mz);
		pp->nouveau->UpdateGyrometer(gx,gy,gz);
		pthread_mutex_unlock(&mutex);
		wait_period (&info);
	}

  return NULL;
}


//Tache du thread kalman
static void *thread_cor(void *parametres)
{
	struct periodic_info info;
	struct param *pp = (struct param *)parametres;
	make_periodic((int)(pp->dt*1000000), &info);
	while(1)
	{
		pthread_mutex_lock(&mutex);
		pp->nouveau->Update();
		pthread_mutex_unlock(&mutex);
		wait_period (&info);	
	}

  return NULL;
}

void ahrs::Start(float dt){
	if(started==false){
		this->Set(dt);
		this->Start();
	}
}

struct param p;

void ahrs::Start(){
	if(started==false){
		int s, inheritsched, policy;
		struct sched_param param;
		p.nouveau = this;
		p.dt=this->dt;

		Navdata::init();

		//CONFIGURATION DES OPTIONS

		//######################### MAIN ###########################

		/* Optionally set scheduling attributes of main thread,
	   and display the attributes */
		//Configuration de la politique d'ordonancement et de la priorite du main
		policy = SCHED_FIFO;
		param.sched_priority = 90;
		s = pthread_setschedparam(pthread_self(), policy, &param);
		if (s != 0)
			handle_error_en(s, "pthread_setschedparam");


		//################## THREAD RECUPERATION DATA ##################

		attrp = NULL;
		s = pthread_attr_init(&attr);
		if (s != 0)
			handle_error_en(s, "pthread_attr_init");

		attrp = &attr;
		inheritsched = PTHREAD_EXPLICIT_SCHED;
		s = pthread_attr_setinheritsched(&attr, inheritsched);
		if (s != 0)
			handle_error_en(s, "pthread_attr_setinheritsched");

		//Configuration de la politique d'ordonancement et de la priorite du thread recuperation data
		policy = SCHED_FIFO;
		param.sched_priority = 89;
		s = pthread_attr_setschedpolicy(&attr, policy);
		if (s != 0)
			handle_error_en(s, "pthread_attr_setschedpolicy");

		s = pthread_attr_setschedparam(&attr, &param);
		if (s != 0)
			handle_error_en(s, "pthread_attr_setschedparam");

		/*s = pthread_detach(thread_recup_data);
	if (s != 0)
		handle_error_en(s, "pthread_detach");*/



		//################## THREAD KALMAN ##################

		/* Initialize thread attributes object */
		attrp2 = NULL;
		s = pthread_attr_init(&attr2);

		if (s != 0)
			handle_error_en(s, "pthread_attr_init");

		attrp2 = &attr2;
		inheritsched = PTHREAD_EXPLICIT_SCHED;

		s = pthread_attr_setinheritsched(&attr2, inheritsched);
		if (s != 0)
			handle_error_en(s, "pthread_attr_setinheritsched");

		//Configuration de la politique d'ordonancement et de la priorite des threads secondaires
		policy = SCHED_FIFO;
		param.sched_priority = 88;

		s = pthread_attr_setschedpolicy(&attr2, policy);
		if (s != 0)
			handle_error_en(s, "pthread_attr_setschedpolicy");

		s = pthread_attr_setschedparam(&attr2, &param);
		if (s != 0)
			handle_error_en(s, "pthread_attr_setschedparam");

		/*s = pthread_detach(thread_kalman);
	if (s != 0)
		handle_error_en(s, "pthread_detach");





	s = pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
	if (s != 0)
		handle_error_en(s, "pthread_attr_setcanceltype");*/



		//CREATION DES THREADS
		//Creation du thread recuperation data
		s = pthread_create(&thread_recup_data, attrp, &thread_recup, &p);
		if (s != 0)
			handle_error_en(s, "pthread_create");

		//Creation du thread kalman
		s = pthread_create(&thread_kalman, attrp2, &thread_cor, &p);
		if (s != 0)
			handle_error_en(s, "pthread_create");
		this->started=true;
	}
}



void ahrs::Stop(){
	if(started==true){
		int s;

		//Annulation des threads
		s = pthread_cancel(thread_recup_data);
		if (s != 0)
			handle_error_en(s, "pthread_cancel");
		s = pthread_cancel(thread_kalman);
		if (s != 0)
			handle_error_en(s, "pthread_cancel");

		//DESTRUCTION DES ATTRIBUTS POUR LIBERER LA MEMOIRE
		pthread_attr_destroy(&attr);
		pthread_attr_destroy(&attr2);
		pthread_attr_destroy(attrp);
		pthread_attr_destroy(attrp2);

		this->started=false;
	}
}
