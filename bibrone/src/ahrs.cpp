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
	if(initialized==false){
		std::FILE * file;
		if((file=std::fopen("config.txt","r"))){
			fscanf(file,"%f\n",&gravity);
			for(int i=0;i<3;i++){
				fscanf(file,"%f %f\n",&max_magn[i],&min_magn[i]);
			}
			initialized=true;
		}
		else
			this->Etalonnage();
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
	Q[0][0]=Q[1][1]=0.001;
	Q[1][0]=Q[0][1]=0;
	for(int i=0;i<3;i++){
		for(int j=0;j<2;j++){
			for(int k=0;k<2;k++){
				P[i][j][k]=0;
			}
		}
	}
}

/*
 * Spécifie le temps d'échantillonage
 */
void ahrs::Set(float dt){
	this->dt=dt;
}

/*
 * Permet de spécifier le temps déchantillonage, le coefficient intégrateur et proportionnel du correcteur
 */
void ahrs::Set(float dt, float ki, float kp){
	this->Set(dt);
	this->Ki=ki;
	this->Kp=kp;
}

/*
 * Permet de spécifier le temps déchantillonage, le coefficient intégrateur et proportionnel du correcteur
 * La confiance accordée à l'acceleromètre et magnétomètre et le coefficient de confiance du correcteur total
 */
void ahrs::Set(float dt, float ki, float kp, float ka, float km, float kt){
	this->Set(dt,ki,kp);
	this->Ka=ka;
	this->Km=km;
	this->Kt=kt;
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
		wy = 2 * bx * (q2q3 - q1q4) + 2 * bz * (q1q2 + q3q4);
		wz = 2 * bx * (q1q3 + q2q4) + 2 * bz * (0.5 - q2q2 - q3q3);

		// Error is cross product between estimated direction and measured direction of gravity
		ex = (2*Ka*(ay * vz - az * vy) + 2*Km*(my * wz - mz * wy))/(Ka+Km);
		ey = (2*Ka*(az * vx - ax * vz) + 2*Km*(mz * wx - mx * wz))/(Ka+Km);
		ez = (2*Ka*(ax * vy - ay * vx) + 2*Km*(mx * wy - my * wx))/(Ka+Km);
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
			corr[i]+=temp2[i];
			corr[i]+=temp3[i];
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
	if(!quaternion){
		Qua = ahrs::MatToQua(R);
	}
	return Qua;
}

/*
 * Renvoit la matrice de rotation
 */
std::vector< std::vector<float> > ahrs::GetRotationMatrix(){
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
	return R;
}

/*
 * Permet de choisir la confiance accordée au filtre de kalman
 */
void ahrs::SetKalman(float kal){
	this->SetKalman(true);
	this->Kal=kal;
}

/*
 * Permet de spécifier si kalman est utilisé
 */
void ahrs::SetKalman(bool use){
	kalman_filter=use;
}

/*
 * Permet de spécifier l'utilisation des quaternions dans les calculs
 */
void ahrs::SetQuaternion(bool use){
	quaternion=use;
}

/*
 * Une itération de kalman (Matrice rotation)
 */
std::vector<float> ahrs::kalman_update(std::vector<float> V){
	float Y[3]={0};
	float K[3][2]={{0}};
	float S;
	float R=0.1;
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
	if(!quaternion)
		Qua=this->MatToQua(R);
	float temp=2*Qua[1]*Qua[3]+2*Qua[0]*Qua[2];
	retour[0] = atan2(2*Qua[2]*Qua[3]-2*Qua[0]*Qua[1],2*Qua[0]*Qua[0]+2*Qua[3]*Qua[3]-1);
	retour[1] = -atan(temp/sqrt(1-temp*temp));
	retour[2] = atan2(2*Qua[1]*Qua[2]-2*Qua[0]*Qua[3],2*Qua[0]*Qua[0]+2*Qua[1]*Qua[1]-1);
	for(int i=0;i<3;i++){
		retour[i]*=360/(2*M_PI);
	}
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
	this->SetQuaternion(true);
	int etalonnage=0;
	mx = -Navdata::IMU::Magnetometer::getY();
	my = Navdata::IMU::Magnetometer::getX();
	mz = Navdata::IMU::Magnetometer::getZ();
	max_magn[0]=min_magn[0]=mx;
	max_magn[1]=min_magn[1]=my;
	max_magn[2]=min_magn[2]=mz;
	float angle_min=0;
	float angle_max=0;
	float avancement=0;
	int passage=0;
	printf("Effectuez un à deux tours avec le drone placé à l'horizontale\n");
	for(int i=0;i<40;i++){
		printf("-");
	}
	printf("\r");fflush(stdout);
	gravity=0;
	float iterations=0;
	while(etalonnage<5000){
		for(int i=0;i<5;i++){
			usleep(2000);
			Navdata::update();
			Navdata::IMU::update();
			gx = Navdata::IMU::Gyroscope::getX();
			gy = Navdata::IMU::Gyroscope::getY();
			gz = Navdata::IMU::Gyroscope::getZ();

			ax = Navdata::IMU::Accelerometer::getX();
			ay = -Navdata::IMU::Accelerometer::getY();
			az = -Navdata::IMU::Accelerometer::getZ();

			mx = -Navdata::IMU::Magnetometer::getY();
			my = Navdata::IMU::Magnetometer::getX();
			mz = Navdata::IMU::Magnetometer::getZ();

			this->UpdateAccelerometer(ax,ay,az);
			this->UpdateMagnetometer(mx,my,mz);
			this->UpdateGyrometer(gx,gy,gz);
			if(mx>max_magn[0] || max_magn[0]==0)
				max_magn[0]=mx;
			else if(mx<min_magn[0] || min_magn[0]==0)
				min_magn[0]=mx;
			if(my>max_magn[1] || max_magn[1]==0)
				max_magn[1]=my;
			else if(my<min_magn[1] || min_magn[1]==0)
				min_magn[1]=my;
			if(mz>max_magn[2] || max_magn[2]==0)
				max_magn[2]=mz;
			else if(mz<min_magn[2] || min_magn[2]==0)
				min_magn[2]=mz;
			gravity+=az;
			iterations++;
		}
		if(pow_sum(magn)!=0){
			this->Update();
			float temp=this->GetQuaternion()[0];
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
	gravity/=iterations;
	fprintf(file,"%f\n",gravity);
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

			ax = Navdata::IMU::Accelerometer::getX();
			ay = -Navdata::IMU::Accelerometer::getY();
			az = -Navdata::IMU::Accelerometer::getZ();

			mx = -Navdata::IMU::Magnetometer::getY();
			my = Navdata::IMU::Magnetometer::getX();
			mz = Navdata::IMU::Magnetometer::getZ();

			this->UpdateAccelerometer(ax,ay,az);
			this->UpdateMagnetometer(mx,my,mz);
			this->UpdateGyrometer(gx,gy,gz);
			if(mx>max_magn[0])
				max_magn[0]=mx;
			else if(mx<min_magn[0])
				min_magn[0]=mx;
			if(my>max_magn[1])
				max_magn[1]=my;
			else if(my<min_magn[1])
				min_magn[1]=my;
			if(mz>max_magn[2])
				max_magn[2]=mz;
			else if(mz<min_magn[2])
				min_magn[2]=mz;
		}
		if(pow_sum(magn)!=0){
			this->Update();
			float temp=this->GetRotationMatrix()[2][1];
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
		fprintf(file,"%f %f\n",max_magn[i],min_magn[i]);
	}
	fclose(file);
	printf("\n");
	printf("Etalonnage terminé\n");
	initialized=true;
}
