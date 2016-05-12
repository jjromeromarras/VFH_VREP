#include "VFH.h"


double VFH::GetHm()
{
	return _Hm;
}
double VFH::GetNumberCellX()
{
	return _NumberCellX;
}

double VFH::GetNumberCellY()
{
	return _NumberCellY;
}

void VFH::SetAlpha(float value)
{
	_Alpha = value;
}

void VFH::SetHm(double value)
{
	_Hm = value;
}

void VFH::SetL(int value)
{
	_L = value;
}

void VFH::SetDistant(int value)
{
	_Distant = value;
}

void VFH::SetTurnRateMax(int value)
{
	_TurnRateMax = value;
}

void VFH::SetSizeCell(float value)
{
	_SizeCell = value;
}

void VFH::SetSizeWindowsActive(int value)
{
	_SizeWindowsActive = value;
}

void VFH::SetSmax(int value)
{
	_Smax = value;
}


double VFH::CalcCellX(double PosX)
{
	
	double PosXCellObstacle = (PosX + GetWide()) / 2;
	

	PosXCellObstacle = floor(PosXCellObstacle / GetSizeCell());
	if (PosXCellObstacle<0) PosXCellObstacle = 0;
	if (PosXCellObstacle >= int(GetNumberCellX()))
		PosXCellObstacle = int(GetNumberCellX()) - 1;
	std::stringstream ss;
	ss << " [VFH::CalcCellX] - PosX= " << PosX << " - PosXCellObstacle= " << PosXCellObstacle;
	log.LogDebug(ss.str().c_str());
	ss.str(std::string());

	return PosXCellObstacle;
}

double VFH::CalcCellY(double PosY)
{
	double PosYCellObstacle = (PosY + GetHeight()) / 2;
	

	PosYCellObstacle = floor(PosYCellObstacle / GetSizeCell());
	if (PosYCellObstacle<0) PosYCellObstacle = 0;
	if (PosYCellObstacle >= int(GetNumberCellY()))
		PosYCellObstacle = int(GetNumberCellY()) - 1;

	std::stringstream ss;
	ss << " [VFH::CalcCellX] - PosY= " << PosY << " - PosYCellObstacle= " << PosYCellObstacle;
	log.LogDebug(ss.str().c_str());
	ss.str(std::string());

	return PosYCellObstacle;
}


double VFH::GetTurnRateMax()
{
	return _TurnRateMax;
}
float VFH::GetDistant()
{

	return _robot->GetnoDetectionDistance();
}

int VFH::GetSizeWindowsActive()
{
	return _SizeWindowsActive;
}

int VFH::GetSmax()
{
	return _Smax;
}
int VFH::GetL()
{
	return _L;
}

float VFH::GetAlpha()
{
	return _Alpha;
}

float VFH::CalculateB(float angle)
{
	double range = 0;
	double angulosonar;
	for (int i = 0; i<_robot->GetNumSonar() - 1; i++)
	{
		
		angulosonar = GetAlfa(angle);
		if (angulosonar<0)
			angulosonar = 0 - angulosonar;
		else
			angulosonar = 360 - angulosonar;

		if (angulosonar<angle)
			range = i;
	}
	return range;
}

float VFH::GetSizeCell()
{
	return _SizeCell;
}

float VFH::GetConstantA()
{
	return _ConstantA;
}

float VFH::GetConstantB()
{
	return _ConstantB;
}

void VFH::CreateCertainty()
{
	_CertaintyValues = new double*[int(GetNumberCellX())];
	for (int i = 0; i<(GetNumberCellX()); i++)
		_CertaintyValues[i] = new double[int(GetNumberCellY())];
	for (int i = 0; i<int(GetNumberCellX()); i++)
		for (int j = 0; j<int(GetNumberCellY()); j++)
			_CertaintyValues[i][j] = 0;

	for (int h = 0; h < GetMaxHistory(); h++)
	{
		double ** auxvalues = new double*[int(GetNumberCellX())];
		for (int i = 0; i<(GetNumberCellX()); i++)
			auxvalues[i] = new double[int(GetNumberCellY())];
		for (int i = 0; i<int(GetNumberCellX()); i++)
			for (int j = 0; j<int(GetNumberCellY()); j++)
				auxvalues[i][j] = 0;
		_historycertaintyvalues.insert({ h, auxvalues });
	}

}

void VFH::FreeArray()
{
	free(_PolarObstacle);
	free(_CertaintyValues);
	free(_PolarObstacleDensity);
}

void VFH::FindSectorRightClose(int *KSectorLeft, int *DistantLeft, double *Polar, float Threshold)
{
	*DistantLeft = 0;
	while (Polar[*KSectorLeft]<Threshold)
	{
		*DistantLeft += 1;
		*KSectorLeft += 1;
		if (*KSectorLeft == int(GetNumberSector()))
			*KSectorLeft = 0;
	}
}

void VFH::FindSectorLeftClose(int *KSectorRight, int *DistantRight, double *Polar, float Threshold)
{
	*DistantRight = 0;
	while (Polar[*KSectorRight]<Threshold)
	{
		*DistantRight += 1;
		*KSectorRight -= 1;
		if (*KSectorRight<0)
			*KSectorRight = int(GetNumberSector()) - 1;
	}
}

void VFH::FindSectorRightFree(int *KSectorLeft, int *DistantLeft, double *Polar, float Threshold)
{
	*DistantLeft = 0;
	while (Polar[*KSectorLeft] >= Threshold)
	{
		*DistantLeft += 1;
		*KSectorLeft += 1;
		if (*KSectorLeft == int(GetNumberSector()))
			*KSectorLeft = 0;
	}
}

void VFH::FindSectorLeftFree(int *KSectorRight, int *DistantRight, double *Polar, float Threshold)
{
	*DistantRight = 0;
	while (Polar[*KSectorRight] >= Threshold)
	{
		*DistantRight += 1;
		*KSectorRight -= 1;
		if (*KSectorRight<0)
			*KSectorRight = int(GetNumberSector()) - 1;
	}
}

float VFH::CalcDirecctionObstacle(float PosXCell, float PosYCell, double PosXRobot, double PosYRobot)
{
	float B;
	if (PosXCell != PosXRobot)
	{
		B = atan((PosYCell - PosYRobot) / (PosXCell - PosXRobot));
		B = (180 * B) / PI;
		

		if (PosYCell == PosYRobot)
		{
			if (PosXCell<PosXRobot)
				B = 180;
		}
		else
		{
			// Cuadrantes primeros
			if (PosXCell > PosXRobot)
			{
				if (PosYCell < PosYRobot)
				{
					// cuadrante enre 0 y 90
					B = abs(B);
				}
				else
				{
					// cuadrante entre 270 y 360
					B = 360 - B;
				}
			}
			else // Cuadrantes de atras
			{
				if (PosYCell < PosYRobot)
				{
					// cuadrante enre 90 y 180
					B = 180-B;
				}
				else
				{
					// cuadrante entre 270 y 360
					B = 180 + abs(B);
				}
			}
		}
	}
	else
	{
		if (PosYCell<PosYRobot)
			B = 270;
		else
			B = 90;
	}
	std::stringstream ss;
	ss << " [VFH::CalcDirecctionObstacle] - Calculando Direcction = " << B << " -PosXCell =" << PosXCell << " - PosYCell=" << PosYCell
		<< " - PosXRobot=" << PosXRobot << "- PosYRobot=" << PosYRobot;
	log.LogDebug(ss.str().c_str());

	return B;
}


float VFH::CalcDirecction(float PosXCell, float PosYCell, double PosXRobot, double PosYRobot)
{
	float B;
	

	if (PosXCell != PosXRobot)
	{
		B = atan((PosYCell - PosYRobot) / (PosXCell - PosXRobot));
		B = (180 * B) / PI;
		

		if (PosYCell == PosYRobot)
		{
			if (PosXCell<PosXRobot)
				B = 180;
		}
		else
		{
			if (((PosYCell - PosYRobot)<0) && ((PosXCell - PosXRobot)>0))
				B += 360;
			else
				if (((PosYCell - PosYRobot)<0) || ((PosXCell - PosXRobot)<0))
					B += 180;
		}
	}
	else
	{
		if (PosYCell<PosYRobot)
			B = 270;
		else
			B = 90;
	}
	std::stringstream ss;
	ss << " [VFH::CalcDirecction] - Calculando Direcction = " << B << " -PosXCell =" << PosXCell << " - PosYCell=" << PosYCell
		<< " - PosXRobot=" << PosXRobot << "- PosYRobot=" << PosYRobot;
	log.LogDebug(ss.str().c_str());
	
	return B;
}

void VFH::CalcWindowsActive(double *XCellWindows, double *YCellWindows, double XCellRobot, double YCellRobot)
{
	*XCellWindows = (XCellRobot - (GetSizeWindowsActive() - 1) / 2);
	if (*XCellWindows<0)
		*XCellWindows = 0;

	*YCellWindows = (YCellRobot - (GetSizeWindowsActive() - 1) / 2);
	if (*YCellWindows<0)
		*YCellWindows = 0;

	for (int i = 0; i<static_cast<int>(GetNumberSector()); i++)
		_PolarObstacle[i] = 0;
}

void VFH::SetPolarObstacle(double XCellRobot, double YCellRobot, double PosXRobot, double PosYRobot)
{
	std::stringstream ss;
	ss.str(std::string());
	ss << " [VFH::SetPolarObstacle] - Calculando PolarObstacle ";
	log.LogDebug(ss.str().c_str());

	double XCellWindows, YCellWindows; // Celda de inicio de la ventana activa
	float PosXCell, PosYCell; // Posición en cartesiandas de la celda activa que se esta tratando
	float M, B; // Variables ara el calculo del vector de polarización
	int K = 0; // Variable que almacena el sector a tratar
	float DistantRobotCell; // Distancia entre el robot y la celda activa que se esta tratando

	for (int i = 0; i<static_cast<int>(GetNumberSector()); i++)
		_PolarObstacle[i] = 0;

	// Una vez obtenida la celda de inicio se recorre la vetnana activa para calcular cada M			
			for (int i = 0; i<int(GetNumberCellX()); i++)
			{
				for (int j = 0; j < int(GetNumberCellY()); j++)
				{
					if (_CertaintyValues[i][j]>0)
					{

						DistantRobotCell = sqrt(pow(XCellRobot - i, 2) + pow((YCellRobot - j), 2));
						M = pow(_CertaintyValues[i][j], 2)*(GetConstantA() - GetConstantB()*DistantRobotCell);
						B = CalcDirecction(i, j, XCellRobot, YCellRobot);
						if (M != 0)
						{
							int indx = static_cast<int>(B/GetAlpha());
							_PolarObstacle[indx] = _PolarObstacle[indx] + M;
							
							ss.str(std::string());
							ss << " [VFH::SetPolarObstacle] - Cell[" << i << "," << j << "] - B =" << B << " _CertaintyValues[" << i << "," << j << "]=" << _CertaintyValues[i][j]
								<< " - M =" << M << " - DistantRobotCell=" << DistantRobotCell << " - _PolarObstacle[" << indx << "] = " << _PolarObstacle[indx];
							log.LogDebug(ss.str().c_str());
						}
					}
				}
		}
}


void VFH::SmoothedPolarObstacleDensity()
{
	int Sup, Inf;
	float ValorInitial = 0;
	for (int i = 0; i<int(GetNumberSector()); i++)
	{
		Sup = i;
		Inf = i;
		_PolarObstacleDensity[i] = _PolarObstacle[i];
		for (int j = 0; j<GetL(); j++)
		{
			Sup += 1;
			if (Sup == int(GetNumberSector()))
				Sup = 0;
			Inf -= 1;
			if (Inf<0)Inf = int(GetNumberSector()) - 1;
			_PolarObstacleDensity[i] = _PolarObstacleDensity[i] + _PolarObstacle[Sup] + _PolarObstacle[Inf];
		}
		_PolarObstacleDensity[i] = _PolarObstacleDensity[i] / (2 * GetL() + 1);
	}
	/*for (int i = 0; i < static_cast<int>(GetNumberSector()); i++)
	{
		std::stringstream ss;
		ss.str(std::string());
		ss << " [VFH::SmoothedPolarObstacleDensity] - _PolarObstacleDensity[" << i << "] -  =" << _PolarObstacleDensity[i];
		log.LogDebug(ss.str().c_str());

	}*/
	
}

void VFH::CreateValley(list<StructValley> *ListCandidateValley, int KAux, double *Polar, float Threshold)
{
	int cont = 0;
	while (cont != int(GetNumberSector()))
	{
		StructValley aux;
		aux.NumberSectors = 0;
		KAux++;
		if (KAux == int(GetNumberSector())) KAux = 0;
		cont += 1;
		while ((Polar[KAux] >= Threshold) && (cont != int(GetNumberSector())))
		{
			// recorremos todos los esetores cerrados hasta encontrar le primero abierto
			KAux += 1;
			if (KAux == int(GetNumberSector())) KAux = 0;
			cont += 1;
		}

		if (cont != int(GetNumberSector()))
		{
			std::stringstream ss;
			ss << " [VFH::CreateValley] - Creando Valle ****************************";
			log.LogDebug(ss.str().c_str());

			while ((Polar[KAux]<Threshold) && (cont != int(GetNumberSector())))
			{
				ss.str(std::string());
				ss << " [VFH::CreateValley] - Add Sector  = " << KAux;
				log.LogDebug(ss.str().c_str());

				aux.list.push_back(KAux);
				aux.NumberSectors++;
				KAux++;
				if (KAux == int(GetNumberSector())) KAux = 0;
				cont++;
			}
			ss.str(std::string());
			ss << " [VFH::CreateValley] - Cerrando Valle ----------------------------";
			ListCandidateValley->push_back(aux);
		}
	}
}


void VFH::SetCandidateValley(list<StructValley> *ListCandidateValley, double *Polar, float Threshold)
{
	int Find = 0;
	int KAux = 0;
	int KValle;
	for (int i = 0; (Find == 0) && (i<int(GetNumberSector())); i++)
	{
		if (Polar[i] >= Threshold)
		{
			Find = 1; KValle = i;
		}
	}

	if (Find)
		CreateValley(ListCandidateValley, KValle, Polar, Threshold);
	else
	{
		StructValley aux;
		for (int i = 1; i<int(GetNumberSector()); i++)
			aux.list.push_back(i);
		aux.NumberSectors = int(GetNumberSector());
		ListCandidateValley->push_back(aux);
	}
}

int VFH::FindValley(list<StructValley> *ListValley, int Kn)
{
	list<StructValley>::iterator itr;
	itr = ListValley->begin();
	StructValley aux;
	while (itr != ListValley->end())
	{
		aux = *itr;
		list<int>::iterator itrSector;
		itrSector = aux.list.begin();
		while (itrSector != aux.list.end())
		{
			if (Kn == *itrSector)
			{
				if (aux.NumberSectors >= int(GetSmax()))
					return 1;
				else
					return 0;
			}
			itrSector++;
		}
		itr++;
	}
	return 0;
}


int VFH::NarrowCaseBValley(int KRight, int KLeft)
{
	int KAlfa;	
	if (KRight<KLeft)
		KRight = KRight + GetNumberSector();

	KAlfa = int(floor(double((KRight + KLeft) / 2)));
	if (KAlfa >= GetNumberSector())
		KAlfa = KAlfa - GetNumberSector();

	std::stringstream ss;
	ss << " [VFH::NarrowCaseBValley] - KAlfa= " << KAlfa;
	log.LogDebug(ss.str().c_str());

	return KAlfa;
}

int VFH::NarrowCaseCValley(int DistantRight, int DistantLeft, int Kn, double *Polar, float Threshold)
{
	int KAlfa, Kf;
	if (DistantLeft <= DistantRight)
	{
		Kf = Kn;
		FindSectorLeftClose(&Kf, &DistantLeft, Polar, Threshold);
		Kf = Kf - 1;
		if (Kf<0) Kf = GetNumberSector() - 1;
		if (Kf<Kn)
			Kf += GetNumberSector();
	}
	else
	{
		Kf = Kn;
		FindSectorRightClose(&Kf, &DistantRight, Polar, Threshold);
		Kf += 1;
		if (Kf == GetNumberSector()) Kf = 0;
		if (Kf>Kn)
			Kn = Kf + DistantRight;
	}
	KAlfa = floor(double((Kn + Kf) / 2));
	if (KAlfa >= GetNumberSector())
		KAlfa -= GetNumberSector();
	return KAlfa;
}


int VFH::WideValleyB(int DistantRight, int DistantLeft, int Kn)
{
	int KAlfa;
	int Kf;
	if (DistantRight<DistantLeft)
	{
		Kf = Kn;
		for (int i = 0; i<GetSmax(); i++)
		{
			Kf -= 1;
			if (Kf<0)
				Kf = GetNumberSector() - 1;
		}
		if (Kf>Kn)
			Kn = Kf + GetSmax();
	}
	else
		Kf = Kn + GetSmax();
	KAlfa = floor(double((Kf + Kn) / 2));
	std::stringstream ss;
	ss << " [VFH::WideValley] - KAlfa= " << KAlfa << " - Kn = " << Kn << " - Kf " << Kf;
	log.LogDebug(ss.str().c_str());
	if (KAlfa >= GetNumberSector())
		KAlfa -= GetNumberSector();
	ss.str(std::string());
	ss << " [VFH::WideValley] - KAlfa Final= " << KAlfa ;
	log.LogDebug(ss.str().c_str());


	return KAlfa;
}

int VFH::WideValleyC(int DistantRight, int DistantLeft, int Kn)
{
	int KAlfa;
	int Kf;
	if (DistantLeft<DistantRight)
	{
		Kf = Kn;
		for (int i = 0; i<GetSmax(); i++)
		{
			Kf -= 1;
			if (Kf<0)
				Kf = GetNumberSector() - 1;
		}
		if (Kf>Kn)
			Kn = Kf + GetSmax();
	}
	else
		Kf = Kn + GetSmax();
	KAlfa = floor(double((Kf + Kn) / 2));
	std::stringstream ss;
	ss << " [VFH::WideValley] - KAlfa= " << KAlfa << " - Kn = " << Kn << " - Kf " << Kf;
	log.LogDebug(ss.str().c_str());
	if (KAlfa >= GetNumberSector())
		KAlfa -= GetNumberSector();
	ss.str(std::string());
	ss << " [VFH::WideValley] - KAlfa Final= " << KAlfa;
	log.LogDebug(ss.str().c_str());


	return KAlfa;
}

int VFH::CaseCValley(list<StructValley> *ListValley, int KGoal, double *Polar, float Threshold)
{
	int DistantLeft, DistantRight;
	int KRight = KGoal, KLeft = KGoal;
	int Kn;
	int KAlfa;
	FindSectorLeftFree(&KLeft, &DistantLeft, Polar, Threshold);
	FindSectorRightFree(&KRight, &DistantRight, Polar, Threshold);
	if (DistantRight >= DistantLeft)
		Kn = KLeft;
	else
		Kn = KRight;
	if (FindValley(ListValley, Kn))
		KAlfa = WideValleyC(DistantRight, DistantLeft, Kn);
	else
		KAlfa = NarrowCaseCValley(DistantRight, DistantLeft, Kn, Polar, Threshold);
	

	std::stringstream ss;
	ss << " [VFH::CaseCValley] - valle candidato = " << Kn << " - Smax = " << GetSmax() << " - KLeft = " << KLeft << " - KRight=" << KRight
		<< " -DistantLeft = " << DistantLeft << " - DistantRight = " << DistantRight << " - Alfa =" << KAlfa*GetAlpha();
	log.LogDebug(ss.str().c_str());


	return KAlfa*GetAlpha();
}

///
/// Caso en el que el sector destino esta dentro de un valle libre
///
int VFH::CaseBValley(StructValley aux, int KGoal, double *Polar, float Threshold)
{
	int DistantLeft, DistantRight;
	int KRight = KGoal, KLeft = KGoal;
	int Kn;
	int KAlfa;
	int Find = 0;
	
	FindSectorLeftClose(&KLeft, &DistantLeft, Polar, Threshold);
	FindSectorRightClose(&KRight, &DistantRight, Polar, Threshold);
	

	if (aux.NumberSectors >= GetSmax())
	{
		if (DistantRight >= DistantLeft)
			Kn = KLeft;
		else
			Kn = KRight;
		KAlfa = WideValleyB(DistantRight, DistantLeft, Kn);
	}
	else
		KAlfa = NarrowCaseBValley(KRight, KLeft);
	std::stringstream ss;
	ss << " [VFH::CaseBValley] - Tamaño valle = " << aux.NumberSectors << " - Smax = " << GetSmax() << " - KLeft = " << KLeft << " - KRight=" << KRight
		<< " -DistantLeft = " << DistantLeft << " - DistantRight = " << DistantRight << " - Alfa ="<< KAlfa*GetAlpha();
	log.LogDebug(ss.str().c_str());
	return KAlfa*GetAlpha();
}

float VFH::SelectValley(double PosXRobot, double PosYRobot, list<StructValley> *ListValley, double *Polar, float Threshold)
{
	int KGoal;


	list<StructValley>::iterator itr;
	StructValley aux;
	std::stringstream ss;
	
	KGoal = static_cast<int>(GetAngleGoal()/GetAlpha());
	if (KGoal >= GetNumberSector()) KGoal -= GetNumberSector();
	
	ss << " [VFH::SelectValley] - BGoal= " << GetAngleGoal() <<" KGoal= " << KGoal;
	log.LogDebug(ss.str().c_str());

	itr = ListValley->begin();
	if (!ListValley->empty())
	{
		aux = *itr;
		if (aux.NumberSectors == GetNumberSector())
		{
			ss.str(std::string());
			ss << " [VFH::SelectValley] - NO HAY OBSTACULOS BGoal= " << GetAngleGoal();
			log.LogDebug(ss.str().c_str());
			return GetAngleGoal();
		}
		else
		{
			int Find = 0;
			while ((itr != ListValley->end()) && (Find != 1))
			{
				aux = *itr;
				list<int>::iterator itrSector;
				itrSector = aux.list.begin();
				while ((itrSector != aux.list.end()) && (Find != 1))
				{
					if (KGoal == *itrSector)
						Find = 1;
					itrSector++;
				}
				itr++;
			}
			if (Find)
				return CaseBValley(aux, KGoal, Polar, Threshold);
			else
				return CaseCValley(ListValley, KGoal, Polar, Threshold);
		}
	}
	else
	{
		SetAngleGoal(_robot->GetAngle()+ 180);
		if (GetAngleGoal() > 360)
			SetAngleGoal(GetAngleGoal() - 360);
		
		ss.str(std::string());
		ss << " [VFH::SelectValley] - No tenemos salida intentamos girar en redondo BGoal= " << GetAngleGoal();
		log.LogDebug(ss.str().c_str());
		return GetAngleGoal();
	}
}


void VFH::NextStep(double *Alfa, double *Velocity, double PosXRobot, double PosYRobot)
{
	double XCellRobot = CalcCellX(PosXRobot);
	double YCellRobot = CalcCellY(PosYRobot);
	list<StructValley> ListValley;

	SetPolarObstacle(XCellRobot, YCellRobot, PosXRobot, PosYRobot);
	SmoothedPolarObstacleDensity();
	SetCandidateValley(&ListValley, _PolarObstacleDensity, GetThreshold());

	*Alfa = SelectValley(PosXRobot, PosYRobot, &ListValley, _PolarObstacleDensity, GetThreshold());
	*Velocity = CalcVelocity(_PolarObstacleDensity, int(*Alfa / GetAlpha()));
	std::stringstream ss;
	ss << " [VFH::NextStep] - Alfa = " << *Alfa << " - Velocity = " << *Velocity;
	log.LogDebug(ss.str().c_str());
}

void VFH::MethodVFH(double PosXRobot, double PosYRobot)
{
	double Alfa;
	double Velocity;
	std::stringstream ss;
	ss << " [VFH::MethodVFH] - destObj = " << destObj;
	log.LogDebug(ss.str().c_str());
	if (this->destObj > 0.07)
	{
		NextStep(&Alfa, &Velocity, PosXRobot, PosYRobot);
	}
	else
	{
		Velocity = 0;
		Alfa = 0;
	}
	SetPositionRobot(int(Velocity), Alfa, GetTurnRate());
}

int VFH::Final()
{
	return 0;
}



void VFH::SetNumberSector(int Value)
{
	_NumberSector = Value;

	_PolarObstacleDensity = new double[Value];
	for (int i = 0; i<Value; i++)
	{
		_PolarObstacleDensity[i] = 0;
	}

	_PolarObstacle = new double[Value];
	for (int i = 0; i<Value; i++)
		_PolarObstacle[i] = 0;

	std::stringstream ss;
	ss << " [VFH::SetNumberSector] - SetNumberSector= " << _NumberSector;
	log.LogDebug(ss.str().c_str());

}

void VFH::ProbabilityDistribution(int PosX, int PosY, float ValueSonar)
{
	float Value;
	Value = GetDistant()- ValueSonar;
	_CertaintyValues[PosX][PosY] = Value;
	std::stringstream ss;
	ss << " [VFH::ProbabilityDistribution] - _CertaintyValues[" << PosX << "][" << PosY << "]=" << Value << " - ValueSonar = " << ValueSonar;
	log.LogDebug(ss.str().c_str());
}

float VFH::CalcVelocity(double *Polar, int KAlfa)
{
	double Vaux;
	double MinH;

	if (Polar[KAlfa] <= GetHm())
		MinH = Polar[KAlfa];
	else
		MinH = GetHm();
	Vaux = GetVmax()*(1 - (MinH / GetHm()));

	std::stringstream ss;
	ss << " [VFH::CalcVelocity] - Polar[" << KAlfa << "]=" << Polar[KAlfa] << " - MinH = " << MinH <<" - Vaux = "<< Vaux 
		<<" - GetVmax = "<< GetVmax() << " - GetTurnRate = " << GetTurnRate() <<" - GetTurnRateMax << " << GetTurnRateMax()
		<<" - GetMin ="<<GetVmin()<<" - Final = "<< 1 - ((GetTurnRate() / GetTurnRateMax()));
	log.LogDebug(ss.str().c_str());

	return Vaux*(1 - (GetTurnRate() / GetTurnRateMax())) + GetVmin();
}


VFH::VFH(Robot *robot,int ClientID, simxInt obj)
{
	log.LogInfo("------------------ [Constructor VFH] ------------------");

	_robot = robot;
	_clientID = ClientID;
	SetParams(0.2, 0.2,1);
	objectivo = obj;
	if (this->_clientID > -1)
		simxGetObjectPosition(this->_clientID, objectivo, -1, ori_obj, simx_opmode_streaming);

	SetSmax(12);
	
	SetVmax(3);
	SetVmin(1);
	SetTurnRateMax(5);
	_TurnRate = 3;

	SetSizeWindowsActive(50);
	SetSizeCell(0.004);
	SetDistant(robot->GetnoDetectionDistance());
	_Threshold = 0.000000001;
	
	SetConstantA(1 + ((GetSizeWindowsActive() - 1) / 2));
	SetConstantB(1);
	SetMaxHistory(25);
	SetThresholdError(10);
	SetL(4);
	SetHm(0.03);
	
	SetAlpha(10);
	double LongX = GetWide()*GetResolution();
	double LongY = GetHeight()*GetResolution();
	SetNumberCellX(LongX / GetSizeCell());
	SetNumberCellY(LongY / GetSizeCell());
	CreateCertainty();
	SetNumberSector(static_cast<int>(360/GetAlpha()));
	indxhist = 0;
	std::stringstream ss;
	ss << " [VFH::VFH] - LongX= " << LongX;
	log.LogDebug(ss.str().c_str());


}



float VFH::GetAlfa(int sector)
{
	switch (sector)
	{
		case 0:
			return 0;
			break;
		case 7:
			return 45;
			break;
		case 6:
			return 90;
			break;
		case 5:
			return 135;
			break;
		case 4:
			return 180;
			break;
		case 3:
			return 225;
			break;
		case 2:
			return 270;
			break;
		case 1:
			return 315;
			break;
	}
}

void VFH::CalcCertaintyValues(double ValueRange, double PosXRobot, double PosYRobot, int Proxy)
{
	double PosXObstacle, PosYObstacle;
	double PosXCellObstacle, PosYCellObstacle;
	double XCellRobot = CalcCellX(PosXRobot);
	double YCellRobot = CalcCellY(PosYRobot);
	std::stringstream ss;
	ss << " [VFH::CalcCertaintyValues] - XCellRobot= " << XCellRobot<<" - YCellRobot= "<< YCellRobot;
	log.LogDebug(ss.str().c_str());
	ss.str(std::string());
	SetObstacle(&PosXObstacle, &PosYObstacle, PosXRobot, PosYRobot, ValueRange, Proxy);
	PosXCellObstacle = CalcCellX(PosXObstacle);
	PosYCellObstacle = CalcCellY(PosYObstacle);
	ss << " [VFH::CalcCertaintyValues] - PosXCellObstacle= " << PosXCellObstacle << " - PosYCellObstacle= " << PosYCellObstacle;
	log.LogDebug(ss.str().c_str());
	ProbabilityDistribution(int(PosXCellObstacle), int(PosYCellObstacle), ValueRange);
}

void VFH::SetConstantA(float value)
{
	_ConstantA = value;
}

void VFH::SetConstantB(float value)
{
	_ConstantB = value;
}

void VFH::SetNumberCellX(double value)
{	
	_NumberCellX = value;
	std::stringstream ss;
	ss << " [VFH::SetNumberCellX] - _NumberCellX= " << _NumberCellX;
	log.LogDebug(ss.str().c_str());
}

void VFH::SetNumberCellY(double value)
{
	_NumberCellY = value;
	std::stringstream ss;
	ss << " [VFH::SetNumberCellY] - _NumberCellY= " << _NumberCellY;
	log.LogDebug(ss.str().c_str());
}

void VFH::UpdateHistoryCertaintyValues()
{
	int i = 0;
	for (const auto& p : _historycertaintyvalues) {
		if (i == indxhist)
		{
			std::stringstream ss;
			ss << " [VFH::UpdateHistoryCertaintyValues] - UpdateHistoryCertaintyValues= " << indxhist;
			log.LogDebug(ss.str().c_str());

			double **auxcert = p.second;
			for (int i = 0; i<int(GetNumberCellX()); i++)
				for (int j = 0; j<int(GetNumberCellY()); j++)
					auxcert[i][j] = _CertaintyValues[i][j];
			indxhist++;
			if (indxhist > GetMaxHistory()-1)
				indxhist = 0;
			break;

		}
		else
			i++;
	}
}


void VFH::UpdateCertaintyValues()
{
	std::stringstream ss;
	ss << " [VFH::UpdateCertaintyValues] - UpdateCertaintyValues";
	log.LogDebug(ss.str().c_str());

	for (const auto& p : _historycertaintyvalues) {
			double **auxcert = p.second;
			for (int i = 0; i<int(GetNumberCellX()); i++)
				for (int j = 0; j<int(GetNumberCellY()); j++)
					_CertaintyValues[i][j] = _CertaintyValues[i][j]+auxcert[i][j] ;
	}
}
void VFH::Run()
{
	double PosXRobot, PosYRobot;	

	while (simxGetConnectionId(_clientID) != -1)
	{		

		log.LogDebug("<-------------------STEP EXECUTE RUN------------------->");
		_robot->UpdatePosition();
		SetAngleRobot(_robot->GetAngle());
		
		
		simxInt errorCode = simxGetObjectPosition(this->_clientID, objectivo, -1, ori_obj, simx_opmode_buffer);
		destObj = INT16_MAX;
		if (errorCode != simx_return_ok)
		{
			SetAngleGoal(0);
		}
		else
		{
			simxFloat B = CalcDirecctionObstacle(ori_obj[0], ori_obj[1], _robot->GetXPos(), _robot->GetYPos());
			SetAngleGoal(B);
			destObj = sqrt(pow(_robot->GetXPos() - ori_obj[0], 2) + pow(_robot->GetYPos() - ori_obj[1], 2));
		}
		
		for (int i = 0; i<int(GetNumberCellX()); i++)
			for (int j = 0; j<int(GetNumberCellY()); j++)
				_CertaintyValues[i][j] = 0;				

		PosXRobot = 0;
		PosYRobot = 0;
	
		for (int i = 0; i < _robot->GetNumSonar(); i++)
		{
			simxFloat value = _robot->GetSensorValue(i);
			if (value < GetDistant())
			{							
				CalcCertaintyValues(value, PosXRobot, PosYRobot, i);
				UpdateHistoryCertaintyValues();
				UpdateCertaintyValues();
			}			
		}
		
		/*log.LogDebug("[VFH::RUN] - CertaintyValues: ");
		log.LogDebug("----------------------------------------------------------------------------");
		std::stringstream ss;
		for (int i = 0; i<int(GetNumberCellY()); i++)
		{
			for (int j = 0; j<int(GetNumberCellX()); j++)
			{
				if(_CertaintyValues[j][i]>0)
					ss << " " <<_CertaintyValues[j][i];				
				else
					if (j== int(GetNumberCellX()/2) && i== int(GetNumberCellY()/2))
						ss << " Ro ";
					else
						ss << " -- " ;
			}

			log.LogDebug(ss.str().c_str());
			ss.str(std::string());

		}
		log.LogDebug("----------------------------------------------------------------------------");*/

		MethodVFH(PosXRobot, PosYRobot);
		//extApi_sleepMs(50);
		log.LogDebug("<********************* STEP FINISH RUN **********************>");

	}

}