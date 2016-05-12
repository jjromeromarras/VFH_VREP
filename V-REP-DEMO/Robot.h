#pragma once

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <windows.h>

#include "MyLogger.h"

extern "C" {
#include "extApi.h"
	/*#include "extApiCustom.h" if you wanna use custom remote API functions! */
}
class Robot
{
private:
	// Handlers para los elementos del robot en V-REP
	simxInt bodyElements, rightMotor, leftMotor;
	simxInt sensor_infra[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
	simxFloat noDetectionDistance;
	// Variables para el almacenamiento de la posición y orientación del robot y posición del objetivo
	simxFloat pos[3], obj[3], ori_cuerpo[3];
	// Identificador del cliente de simulación
	int clientId;
	MyLogger log;
public:
#pragma region Constructor
	Robot(simxInt,simxInt,simxInt,int);
	~Robot();
#pragma endregion

#pragma region SetMethods
	void SetVelocityRightMotor(simxFloat);
	void SetVelocityLeftMotor(simxFloat);
	void SetSensorInfra(simxInt, simxInt);
	void UpdatePosition();
#pragma endregion

#pragma region GetMethods
	simxFloat GetSensorValue(simxInt);
	simxFloat GetXPos();
	simxFloat GetYPos();
	simxFloat GetAngle();
	simxFloat GetnoDetectionDistance();
	int GetNumSonar();
#pragma endregion 
};

