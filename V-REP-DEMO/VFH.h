#pragma once
#include "Algoritmo.h"
#include "Datos.h"
#include "Robot.h"
#include <map>

extern "C" {
#include "extApi.h"
	/*#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

typedef double **CValues;

using namespace std;


class VFH :public Algoritmo
{
private:
#pragma region Attributes
	CValues _CertaintyValues;	
	map<int, CValues> _historycertaintyvalues;
	int indxhist;
	double *_PolarObstacleDensity;
	double *_PolarObstacle;
	double _Hm;
	double _NumberCellX, _NumberCellY;
	int _SizeWindowsActive;
	int _Distant;
	int _Smax;
	double _TurnRateMax;
	int _L;
	float _Alpha;
	float _SizeCell;
	float _ConstantA, _ConstantB;
	int _clientID;
	simxInt objectivo;
	simxFloat ori_obj[3];
	simxFloat destObj;
#pragma endregion

public:
#pragma region Constructor
	VFH(Robot *,int, simxInt);
#pragma endregion

#pragma region GetMethods
	double GetHm();
	double GetNumberCellX();
	double GetNumberCellY();
	double CalcCellX(double);
	double CalcCellY(double);
	double GetTurnRateMax();
	float GetDistant();
	int GetSizeWindowsActive();
	int GetSmax();
	int GetL();
	float GetAlpha();
	float CalculateB(float);
	float GetSizeCell();
	float GetConstantA();
	float GetConstantB();
	
#pragma endregion

#pragma region SetMethods
	void SetAlpha(float);
	void SetDistant(int);
	void SetL(int);
	void SetSizeWindowsActive(int);
	void SetConstantA(float);
	void SetConstantB(float);
	void SetSizeCell(float);
	void SetSmax(int);
	void SetNumberCellX(double);
	void SetNumberCellY(double);
	void SetTurnRateMax(int);
	void SetHm(double);
#pragma endregion

private:
#pragma region VFHMethods
	void CreateCertainty();
	void FreeArray();
	void NextStep(double*, double*, double, double);
	void SetNumberSector(int);
	int WideValleyB(int, int, int);
	int WideValleyC(int, int, int);
	int NarrowCaseCValley(int, int, int, double *, float);
	int NarrowCaseBValley(int, int);
	float CalcDirecction(float, float, double, double);
	float CalcDirecctionObstacle(float, float, double, double);
	void SmoothedPolarObstacleDensity();
	void SetPolarObstacle(double, double, double, double);
	void CalcCertaintyValues(double, double, double, int);
	void FindSectorRightFree(int*, int*, double*, float);
	void FindSectorLeftFree(int*, int*, double*, float);
	void FindSectorRightClose(int*, int*, double*, float);
	void FindSectorLeftClose(int*, int*, double*, float);
	void ProbabilityDistribution(int, int, float);
	void CreateValley(list<StructValley>*, int, double*, float);
	void CalcWindowsActive(double*, double*, double, double);
	int FindValley(list<StructValley>*, int);
	float CalcVelocity(double *, int);
	float SelectValley(double, double, list<StructValley>*, double*, float);
	int CaseBValley(StructValley, int, double*, float);
	int CaseCValley(list<StructValley>*, int, double*, float);
	void MethodVFH(double, double);
	void SetCandidateValley(list<StructValley>*, double*, float);
	void UpdateHistoryCertaintyValues();
	void UpdateCertaintyValues();
	int Final();
	float GetAlfa(int);
#pragma endregion
public:
	void Run();
};

