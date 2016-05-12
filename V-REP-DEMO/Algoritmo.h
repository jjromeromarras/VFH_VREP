#pragma once
#include "Robot.h"
#include "MyLogger.h"

const double PI = 3.1415926;

class Algoritmo
{
protected:

#pragma region Attributes V-REP
	Robot *_robot;
#pragma endregion

#pragma region Attributes VFF
	float _AngleRobot;
	float _AngleGoal;
	float _Vmax, _Vmin;
	float _XRobot, _YRobot;
	float _XGoal, _YGoal;
	double _NumberSector;
	float _Threshold;
	float _ThresholdError;
	double _TurnRate;
	int _maxhistory;
	float _height, _wide, _resolution;
	MyLogger log;
#pragma endregion

public:
	Algoritmo();
	~Algoritmo();

#pragma region SetMethods
	void SetAngleRobot(float);
	void SetVmax(float);
	void SetVmin(float);
	void SetXRobot(float);
	void SetYRobot(float);
	void SetXGoal(float);
	void SetYGoal(float);
	void SetPositionRobot(float, float, float);
	void SetAngleGoal(float);
	void SetMaxHistory(int);
	void SetParams(float, float, float);
	void SetThresholdError(float);
#pragma endregion

#pragma region GetMethods
	float GetVmax();
	float GetVmin();
	float GetXRobot();
	float GetYRobot();
	float GetXGoal();
	float GetYGoal();
	float GetAngleReal();
	float GetAngleRobot();
	float GetAngleGoal();
	float GetThreshold();
	float GetWide();
	float GetResolution();
	float GetHeight();
	double GetTurnRate();
	int GetMaxHistory();
	double GetNumberSector();
	float GetThresholdError();
#pragma endregion 

#pragma region VFFMethods
	void InitialVFF();
#pragma endregion
protected:
#pragma region VFFMethods
	void SetObstacle(double *, double *, double, double, double, int);
	virtual float GetAlfa(int) = 0;
#pragma endregion
};

