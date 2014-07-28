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
	printf("+---------");
	for(unsigned int i=1;i<V[0].size();i++){
		printf("+---------");
	}
	printf("+\n");
	for(unsigned int i=0;i<V.size();i++){
		for(unsigned int j=0;j<V[i].size();j++){
			printf("│%9.3g",V[i][j]);
		}
		printf("│\n");
		if(i+1<V.size()){
			printf("+---------");
			for(unsigned int j=1;j<V[i].size();j++){
				printf("+---------");
			}
			printf("+\n");
		}
	}
	printf("+---------");
	for(unsigned int i=1;i<V[V.size()-1].size();i++){
		printf("+---------");
	}
	printf("+\n");
}

void printR(int n, std::vector<float> V){
	printf("%*d \n",(int)V.size()*6,n);
	printf("+---------");
	for(unsigned int i=1;i<V.size();i++){
		printf("+---------");
	}
	printf("+\n");
	for(unsigned int j=0;j<V.size();j++){
		printf("│%9.3g",V[j]);
	}
	printf("│\n");

	printf("+---------");
	for(unsigned int i=1;i<V.size();i++){
		printf("+---------");
	}
	printf("+\n");
}

int main(){
	ahrs test;
	test.Initialize();
	test.Start(0.01);
	int j=0;
	while(j<500){
		usleep(100000);
		printR(j,test.GetQuaternion());
		printR(j,test.GetRotationMatrix());
		j++;
	}
}
