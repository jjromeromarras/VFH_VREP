#include "Robot.h"



Robot::Robot(simxInt body, simxInt leftmotor, simxInt rightmotor, int clientId)
{
	this->bodyElements = body;
	this->leftMotor = leftmotor;
	this->rightMotor = rightmotor;
	this->clientId = clientId;
	this->noDetectionDistance = 0.2;
	if (log.Init())
	{
		log.LogInfo("------------------ [Constructor Robot] ------------------");
		if (this->clientId > -1)
		{
			simxInt errorcode = simxGetObjectOrientation(this->clientId, bodyElements, -1, ori_cuerpo, simx_opmode_streaming);
			std::stringstream ss;
			ss <<" [Robot] ["<< errorcode<<"] Initial simxGetObjectOrientation";
			log.LogDebug(ss.str().c_str());
			ss.str(std::string());
			errorcode = simxGetObjectPosition(this->clientId, bodyElements, -1, pos, simx_opmode_streaming);
			ss << " [Robot] [" << errorcode << "] Initial simxGetObjectPosition";
			log.LogDebug(ss.str().c_str());
		}
	}
}


Robot::~Robot()
{
}

////////////////////////////  Métodos Get ////////////////////////////  
simxFloat Robot::GetSensorValue(simxInt indx)
{

	simxUChar detectionState;
	simxFloat detectedPoint[3];
	simxInt detectedObjectHandle;
	simxFloat detectedSurfaceNormalVector[3];
	simxInt errorCode = simx_return_ok;
	errorCode = simxReadProximitySensor(this->clientId, this->sensor_infra[indx], &detectionState,
		detectedPoint, &detectedObjectHandle, detectedSurfaceNormalVector, simx_opmode_buffer);

	if (detectionState != 0 && errorCode == simx_return_ok)
	{
		// something was detected
		std::stringstream ss;
		ss << " [Robot::GetSensorValue] [" << errorCode << "] - Sensor["<< indx<<"]="<< detectedPoint[2];
		log.LogDebug(ss.str().c_str());
		
		return detectedPoint[2];
	}
	else
	{	
		return noDetectionDistance;
	}
}

simxFloat Robot::GetXPos()
{
	return pos[0];
}

simxFloat Robot::GetYPos()
{
	return pos[1];
}

simxFloat Robot::GetAngle()
{

	simxFloat angle;
	if (ori_cuerpo[2] < 0)
		// Nos encontramos entre 0 y 180
		angle = ((ori_cuerpo[2] * 90) / -1.5);
	else
		angle = 360 - ((ori_cuerpo[2] * 90) / 1.5);

	std::stringstream ss;
	ss << " [Robot::GetAngle] - angle=" << angle;
	log.LogDebug(ss.str().c_str());

	return angle;
}

int Robot::GetNumSonar()
{
	return 8;
}

simxFloat Robot::GetnoDetectionDistance()
{
	return noDetectionDistance;
}

////////////////////////////  Métodos Set ////////////////////////////  
void Robot::UpdatePosition()
{
	simxInt errorCode = simxGetObjectPosition(this->clientId, bodyElements, -1, pos, simx_opmode_buffer);
	std::stringstream ss;
	ss << " [Robot::UpdatePosition][" << errorCode<<"]";
	log.LogDebug(ss.str().c_str());


	if (errorCode != simx_return_ok)
	{
		pos[0] = pos[1] = 0;
	}
	ss.str(std::string());	
	ss << " [Robot::UpdatePosition] - XPos=" << pos[0];
	log.LogDebug(ss.str().c_str());
	ss.str(std::string());
	ss << " [Robot::UpdatePosition] - YPos=" << pos[1];
	log.LogDebug(ss.str().c_str());

	errorCode = simxGetObjectOrientation(this->clientId, bodyElements, -1, ori_cuerpo, simx_opmode_buffer);
	if (errorCode != simx_return_ok)
	{
		ori_cuerpo[2] = 0;
	}

	ss.str(std::string());
	ss << " [Robot::UpdatePosition] - Angle=" << ori_cuerpo[2];
	log.LogDebug(ss.str().c_str());
}

void Robot::SetSensorInfra(simxInt indx, simxInt sensor)
{
	this->sensor_infra[indx] = sensor;	
	if (this->clientId > -1)
	{
		simxUChar detectionState;
		simxFloat detectedPoint[3];
		simxInt detectedObjectHandle;
		simxFloat detectedSurfaceNormalVector[3];
		simxInt errorCode = simxReadProximitySensor(this->clientId, sensor_infra[indx], &detectionState,
			detectedPoint, &detectedObjectHandle, detectedSurfaceNormalVector, simx_opmode_streaming);
		
		
		simxFloat posSensor[3];
		errorCode = simxGetObjectOrientation(this->clientId, sensor_infra[indx], -1, posSensor, simx_opmode_streaming);
		
		std::stringstream ss;
		ss << " [Robot::SetSensorInfra] - Initial Sensor["<<indx<<"]=" << sensor;
		log.LogDebug(ss.str().c_str());
	}
}

void Robot::SetVelocityRightMotor(simxFloat v)
{	
	simxInt errorCode = simxSetJointTargetVelocity(this->clientId, this->rightMotor, v, simx_opmode_oneshot);
	std::stringstream ss;
	ss << " [Robot::SetVelocityRightMotor] - Set Velocity = " << v ;
	log.LogDebug(ss.str().c_str());
}

void Robot::SetVelocityLeftMotor(simxFloat v)
{
	simxInt errorCode = simxSetJointTargetVelocity(this->clientId, this->leftMotor, v, simx_opmode_oneshot);
	std::stringstream ss;
	ss << " [Robot::SetVelocityLeftMotor] - Set Velocity = " << v;
	log.LogDebug(ss.str().c_str());

}

