/*
 * ahrs.h
 *
 *  Created on: Jun 23, 2014
 *      Author: ohachett
 */

#include <stdio.h>
#include <vector>
#include <math.h>
#include <float.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/timerfd.h>


#define handle_error_en(en, msg) \
    do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)


#ifndef AHRS_HPP_
#define AHRS_HPP_

#define AHRS_HPP_MX -1
#define AHRS_HPP_MY 1
#define AHRS_HPP_MZ -1

#define AHRS_HPP_AX -1
#define AHRS_HPP_AY -1
#define AHRS_HPP_AZ -1

/**
 * La classe ahrs prends les données de capteurs en entrée et retourne les angles d'Euler
 */
class ahrs{
public:
	ahrs() :
		R(3,std::vector<float>(3,0.0)),
		Q(2,std::vector<float>(2,0.0)),
		P(3,std::vector< std::vector<float> >(2,std::vector<float>(2,0.0))),
		W(3,std::vector<float>(2,0.0)),
		corr(3,0.0),
		corri(3,0.0),
		Qua(4,0.0),
		accel(3,0.0),
		magn(3,0.0),
		gyro(3,0.0),
		old(3,0.0)
	{
		/*
		 * Empiriquement Kp=1 Ki=0.5
		 */
		dt=0.010;
		Kp=100;
		Ki=5;
		Ka=1;
		Km=0.2;
		Kt=1;
		Kal=1;
		for(int i=0;i<3;i++)
			R[i][i]=1;
		Qua[0]=1;
		kalman_filter=true;
		quaternion=true;
		gravity=1;
		initialized=false;
		started=false;
	};
	//A lancer au moins une fois avant chaque série d'utilisation
	//Ecrit dans le fichier "config.txt"
	void Etalonnage();
	//Permettent de lancer l'AHRS en tâche de fond
	void Start();
	void Stop();
	//Permet de spécifier l'interval de mise à jour de l'AHRS (en secondes)
	void Start(float dt);
	//Autres fonctions, voir documentation
	void Update(float mx, float my, float mz, float ax, float ay, float az, float gp, float gq, float gr);
	void UpdateMagnetometer(float mx, float my, float mz);
	void UpdateAccelerometer(float ax, float ay, float az);
	void UpdateGyrometer(float gp, float gq, float gr);
	void Update();
	void Initialize();
	void Set(float dt);
	void Set(float dt, float ki, float kp);
	void Set(float dt, float ki, float kp, float ka, float km, float kt);
	void SetKalman(float kal);
	void SetKalman(bool);
	void SetQuaternion(bool);
	std::vector< std::vector<float> > GetRotationMatrix();
	std::vector<float> GetQuaternion();
	std::vector<float> GetEuler();
private:
	std::vector<float> cross_product(std::vector<float>, std::vector<float>);
	std::vector< std::vector<float> > multiply(std::vector< std::vector<float> >,std::vector< std::vector<float> >);
	float pow_sum(std::vector<float>);
	std::vector<float> kalman_update(std::vector<float>);
	std::vector<float> MatToQua(std::vector< std::vector<float> >);

	bool kalman_filter;
	bool quaternion;
	bool initialized;
	bool started;

	float Kp;
	float Ki;
	float Ka;
	float Km;
	float Kt;
	float Kal;
	float dt;

	float max_magn[3],min_magn[3];
	float gravity;

	std::vector< std::vector<float> > R;
	std::vector< std::vector<float> > Q;
	std::vector< std::vector< std::vector<float> > > P;
	std::vector< std::vector<float> > W;
	std::vector<float> corr;
	std::vector<float> corri;
	std::vector<float> Qua;
	std::vector<float> accel;
	std::vector<float> magn;
	std::vector<float> gyro;
	std::vector<float> old;
};
#endif /* AHRS_HPP_ */
