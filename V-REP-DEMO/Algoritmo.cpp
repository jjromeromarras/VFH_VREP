#include "Algoritmo.h"
#include <Math.h>


Algoritmo::Algoritmo()
{
	log.Init();
}


Algoritmo::~Algoritmo()
{
}

void Algoritmo::SetAngleRobot(float Angle)
{
	_AngleRobot = Angle;
	std::stringstream ss;
	ss << " [Algoritmo::SetAngleRobot] - Angle= "<< Angle;
	log.LogDebug(ss.str().c_str());

}


void Algoritmo::SetThresholdError(float value)
{
	_ThresholdError = value;
}

void Algoritmo::SetVmax(float Vmax)
{
	_Vmax = Vmax;
}

void Algoritmo::SetVmin(float Vmin)
{
	_Vmin = Vmin;
}

void Algoritmo::SetXRobot(float X)
{
	_XRobot = X;
}

void Algoritmo::SetYRobot(float Y)
{
	_YRobot = Y;
}

void Algoritmo::SetXGoal(float value)
{
	_XGoal = value;
}

void Algoritmo::SetYGoal(float value)
{
	_YGoal = value;
}

void Algoritmo::SetMaxHistory(int value)
{
	_maxhistory = value;
}

void Algoritmo::SetAngleGoal(float value)
{
	std::stringstream ss;
	ss << " [Algoritmo::SetAngleGoal] - value= " << value;
	log.LogDebug(ss.str().c_str());
	_AngleGoal = value;
}

void Algoritmo::SetParams(float HeightValue, float WideValue, float ResolutionValue)
{
	_height = HeightValue;
	_wide = WideValue;
	_resolution = ResolutionValue;
}

float Algoritmo::GetWide()
{
	return _wide;
}

float Algoritmo::GetResolution()
{
	return _resolution;
}

float Algoritmo::GetThresholdError()
{
	return _ThresholdError;
}

float Algoritmo::GetHeight()
{
	return _height;
}

int Algoritmo::GetMaxHistory()
{
	return _maxhistory;
}

float Algoritmo::GetVmax()
{
	return _Vmax;
}

float Algoritmo::GetVmin()
{
	return _Vmin;
}
float Algoritmo::GetXGoal()
{
	return _XGoal;
}

float Algoritmo::GetThreshold()
{
	return _Threshold;
}

double Algoritmo::GetTurnRate()
{
	return _TurnRate;
}

double Algoritmo::GetNumberSector()
{
	return _NumberSector;
}
float Algoritmo::GetYGoal()
{
	return _YGoal;
}
float Algoritmo::GetXRobot()
{
	std::stringstream ss;
	ss << " [Algoritmo::GetXRobot] - _XRobot= " << _robot->GetXPos();
	log.LogDebug(ss.str().c_str());
	return _robot->GetXPos();
}

float Algoritmo::GetYRobot()
{
	std::stringstream ss;
	ss << " [Algoritmo::GetXRobot] - _YRobot= " << _robot->GetYPos();
	log.LogDebug(ss.str().c_str());
	return _robot->GetYPos();
}

void Algoritmo::InitialVFF()
{

	if (_robot == NULL)
	{
		//ArLog::log(//ArLog::Terse, "Algoritmo: InitialVFF: Warning: I found no robot, deactivating.");
		// deactivate();
	}
}

float Algoritmo::GetAngleRobot()
{
	return this->_AngleRobot;
}

float Algoritmo::GetAngleReal()
{
	/*double angle;
	angle = float(_robot->GetAngle() + GetAngleRobot());
	if (angle>360)angle = angle - 360;
	return angle;*/
	return 0;
}

float Algoritmo::GetAngleGoal()
{
	return _AngleGoal;
}

void Algoritmo::SetPositionRobot(float Velocity, float Alfa, float TurnRate)
{
	double AngleReal = _robot->GetAngle();
	bool avanzar = false;
	float turnVelocity;

	if (AngleReal == Alfa)
	{
		// Vamos recto
		_robot->SetVelocityLeftMotor(Velocity);
		_robot->SetVelocityRightMotor(Velocity);
		
	}
	else
	{
		
		if (AngleReal <= 180)
		{
			if (Alfa < 180)
			{			
				if (abs(Alfa - AngleReal) <= GetThresholdError())
				{
					// Vamos recto
					_robot->SetVelocityLeftMotor(Velocity);
					_robot->SetVelocityRightMotor(Velocity);
				}
				else if (Alfa < AngleReal)
				{
					
					// Giramos a la izquierda
					_robot->SetVelocityLeftMotor(Velocity - TurnRate);
					_robot->SetVelocityRightMotor(Velocity + TurnRate);
				}
				else
				{
					// Giramos a la derecha
					_robot->SetVelocityLeftMotor(Velocity+ TurnRate);
					_robot->SetVelocityRightMotor(Velocity - TurnRate);
				}
			}
			else
			{				
				if (abs(Alfa -(AngleReal+180))<= GetThresholdError())
				{
					// Vamos recto
					_robot->SetVelocityLeftMotor(Velocity);
					_robot->SetVelocityRightMotor(Velocity);
				}
				else
				if (Alfa <= (AngleReal + 180))
				{
					_robot->SetVelocityLeftMotor(Velocity + TurnRate);
					_robot->SetVelocityRightMotor(Velocity - TurnRate);
				}
				else
				{
					_robot->SetVelocityLeftMotor(Velocity - TurnRate);
					_robot->SetVelocityRightMotor(Velocity + TurnRate);
				}
			}
		}
		else
		{
			if (Alfa > (AngleReal) || (Alfa <= (AngleReal + 180) - 360))
			{				
				if (
					(Alfa > AngleReal && abs(Alfa- AngleReal) <= GetThresholdError())
					|| 
					((Alfa <= (AngleReal + 180) - 360) && (abs(Alfa-(AngleReal + 180) - 360))<=10)
					)
				{
					// Vamos recto
					_robot->SetVelocityLeftMotor(Velocity);
					_robot->SetVelocityRightMotor(Velocity);
				}
				else
				{
					// Giramos a la derecha
					_robot->SetVelocityLeftMotor(Velocity + TurnRate);
					_robot->SetVelocityRightMotor(Velocity - TurnRate);
				}
			}
			else
			{
				_robot->SetVelocityLeftMotor(Velocity - TurnRate);
				_robot->SetVelocityRightMotor(Velocity + TurnRate);
			}

		}

	}	
}

void Algoritmo::SetObstacle(double *PosXObstacle, double *PosYObstacle, double PosXRobot, double PosYRobot,
	double ValueSonar, int Sonar)
{
	float AngleSector;
	float ValueX, ValueY;
	
	AngleSector = GetAlfa(Sonar) + _robot->GetAngle();
	if (AngleSector >= 360)AngleSector -= 360;
	if (AngleSector<0)AngleSector += 360;

	std::stringstream ss;


	if ((AngleSector >= 0) && (AngleSector <= 90))
	{
		if (AngleSector == 0)
		{
			ValueX = PosXRobot + double(ValueSonar);
			ValueY = PosYRobot;
		}
		else
			if (AngleSector == 90)
			{
				ValueY = PosYRobot + double(ValueSonar);
				ValueX = PosXRobot;
			}
			else
			{
				ValueX = (cos((AngleSector*PI) / 180))*ValueSonar;
				ValueX = ValueX + PosXRobot;
				ValueY = (sin((AngleSector*PI) / 180))*ValueSonar;
				ValueY = ValueY + PosYRobot;
			}
	}
	else
	{
		if ((AngleSector == 90) || (AngleSector == 180))
		{
			if (AngleSector == 180)
			{
				ValueX = PosXRobot - double(ValueSonar);
				ValueY = PosYRobot;
			}
			else
			{
				ValueX = (sin(((AngleSector - 90)*PI) / 180))*ValueSonar;
				ValueX = PosXRobot - ValueX;
				ValueY = (cos(((AngleSector - 90)*PI) / 180))*ValueSonar;
				ValueY = ValueY + PosYRobot;
			}
		}
		else
		{
			if ((AngleSector>180) && (AngleSector <= 270))
			{
				if (AngleSector == 270)
				{
					ValueY = PosYRobot - double(ValueSonar);
					ValueX = PosXRobot;
				}
				else
				{
					ValueX = (cos(((AngleSector - 180)*PI) / 180))*ValueSonar;
					ValueX = PosXRobot - ValueX;
					ValueY = (sin(((AngleSector - 180)*PI) / 180))*ValueSonar;
					ValueY = PosYRobot - ValueY;
				}
			}
			else
			{
				ValueX = (sin(((AngleSector - 270)*PI) / 180))*ValueSonar;
				ValueX = ValueX + PosXRobot;
				ValueY = (cos(((AngleSector - 270)*PI) / 180))*ValueSonar;
				ValueY = PosYRobot - ValueY;								
			}
		}
	}
	*PosXObstacle = ValueX;
	*PosYObstacle = ValueY;

	
	ss << " [Algoritmo::SetObstacle] - PosXObstacle= " << *PosXObstacle << " - PosYObstacle =" << *PosYObstacle
		<< " - PosXRobot =" << PosXRobot << " - PosYRobot= " << PosYRobot << " - ValueSonar= " << ValueSonar << " - Sonar = " << Sonar;
	log.LogDebug(ss.str().c_str());

	ss.str(std::string());
	ss << " [Algoritmo::SetObstacle] - AngleSector= " << AngleSector;
	log.LogDebug(ss.str().c_str());
	
}