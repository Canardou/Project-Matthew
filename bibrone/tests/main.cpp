#include <bibrone/Actuators.h>
#include <bibrone/Navdata.h>
#include <bibrone/ahrs.h>
#include <../src/ahrs.cpp>
#include <iostream>

#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

#include <algorithm>


#include <vector>
#include <float.h>

void printR(int n, std::vector< std::vector<float> > V){
	printf("%*d \n",(int)V.size()*6,n);
	printf("╭‒‒‒‒‒‒‒‒‒");
	for(unsigned int i=1;i<V[0].size();i++){
		printf("┬‒‒‒‒‒‒‒‒‒");
	}
	printf("╮\n");
	for(unsigned int i=0;i<V.size();i++){
		for(unsigned int j=0;j<V[i].size();j++){
			printf("│%9.3g",V[i][j]);
		}
		printf("│\n");
		if(i+1<V.size()){
			printf("├‒‒‒‒‒‒‒‒‒");
			for(unsigned int j=1;j<V[i].size();j++){
				printf("┼‒‒‒‒‒‒‒‒‒");
			}
			printf("┤\n");
		}
	}
	printf("╰‒‒‒‒‒‒‒‒‒");
	for(unsigned int i=1;i<V[V.size()-1].size();i++){
		printf("┴‒‒‒‒‒‒‒‒‒");
	}
	printf("╯\n");
}

void printR(int n, std::vector<float> V){
	printf("%*d \n",(int)V.size()*6,n);
	printf("╭‒‒‒‒‒‒‒‒‒");
	for(unsigned int i=1;i<V.size();i++){
		printf("┬‒‒‒‒‒‒‒‒‒");
	}
	printf("╮\n");
	for(unsigned int j=0;j<V.size();j++){
		printf("│%9.3g",V[j]);
	}
	printf("│\n");

	printf("╰‒‒‒‒‒‒‒‒‒");
	for(unsigned int i=1;i<V.size();i++){
		printf("┴‒‒‒‒‒‒‒‒‒");
	}
	printf("╯\n");
}

int main(){
	ahrs test;
	test.Initialize();
	test.SetQuaternion(true);
	test.SetKalman(false);
	test.Start(0.01);
	int j=0;
	while(j<100){
		usleep(100000);
		printR(j,test.GetEuler());
		j++;
	}
}

/*int main(){
	FILE * file;
	float gx = 0, gy = 0, gz = 0 ;
	float mx = 0, my = 0, mz = 0 ;
	float ax = 0, ay = 0, az = 0 ;
	//IMU
	Navdata::init();
	//AHRS
	ahrs test;
	test.Initialize();
	test.SetQuaternion(true);
	test.SetKalman(true);
	//Fin d'initialisation
	file=std::fopen("samples.csv","w");
	std::fprintf(file,"mx	my	mz	ax	ay	az	gx	gy	gz	qw	qx	qy	qz	phi	the	psy\n");
	//Boucle de récupérations de données
	int j=0;
	int k=0;
	int i=0;
	while (j<=20000) {
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

		test.UpdateAccelerometer(ax,ay,az);
		test.UpdateMagnetometer(mx,my,mz);
		test.UpdateGyrometer(gx,gy,gz);

		k++;
		if(k==5){
			test.Update();
			k=0;
		}

		//if(j>5000 && j<=10000)
			//std::fprintf(file,"%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f	%f\n"
			//		,mx,my,mz,ax,ay,az,gx,gy,gz,
			//		test.GetQuaternion()[0],test.GetQuaternion()[1],test.GetQuaternion()[2],test.GetQuaternion()[3],
			//		test.GetEuler()[0],test.GetEuler()[1],test.GetEuler()[2]);

		if(i>=100)
		{
			printR(j,test.GetEuler());
			i=0;
		}
		i++;
		j++;
		usleep(2000);
	}
	std::fclose(file);
	return 0;
}*/
