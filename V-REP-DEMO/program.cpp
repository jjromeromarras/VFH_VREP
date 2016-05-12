#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <windows.h>

#include "MyLogger.h"
#include "Robot.h"
#include "VFH.h"

extern "C" {
#include "extApi.h"
	/*#include "extApiCustom.h" if you wanna use custom remote API functions! */
}


int main(int argc, char** argv)
{
	int portNb = 0;   //Se define el puerto de conexión
	simxInt bodyElements, rightMotor, leftMotor,console;
	simxInt sensor_infra[8] = { -1,-1,-1,-1,-1,-1,-1,-1 };
	simxInt objectivo;

	
	MyLogger log;
	if (log.Init())
	{
		log.LogInfo("\n");
		log.LogInfo(" ");
		log.LogInfo(" ");
		log.LogInfo(" ");
		log.LogInfo("**********************************************************************************");
		log.LogInfo("**************************   Inicializando Programa   **************************");
		log.LogInfo("Traza iniciada");
	}
	

	std::stringstream ss;
	ss << "Leyendo Parametros (" << argc << ")...";
	log.LogInfo(ss.str().c_str());

	if (argc >= 15)
	{
		portNb = atoi(argv[1]);
		leftMotor = atoi(argv[2]);
		rightMotor = atoi(argv[3]);
		objectivo = atoi(argv[4]);
		bodyElements = atoi(argv[5]);
		console = atoi(argv[14]); 
		ss.str(std::string());
		ss << "portNb = " << portNb << " leftMotor = "<< leftMotor << " rightMotor = " << rightMotor
			<< " objectivo = " << objectivo << " bodyElements = " << bodyElements<< " console = "<< console;
		log.LogDebug(ss.str().c_str());
		ss.str(std::string());
		for (int i = 0; i < 8; i++)
		{
			sensor_infra[i] = atoi(argv[i + 6]);
			ss << " sensor_infra[" << i << "] = " << sensor_infra[i];
		}
		log.LogDebug(ss.str().c_str());
		
	}
	else
	{
		log.LogInfo("Por favor indicar los parámetros correctos");
		return 0;
	}

	//Conectar con V-REP
	int clientID = simxStart((simxChar*)"127.0.0.1", portNb, true, true, 2000, 5);

	//Si la conexión es exitosa iniciar la simulación
	if (clientID > -1)
	{	

		Robot robot(bodyElements, leftMotor, rightMotor, clientID);
		for (int i = 0; i < 8; i++)
			robot.SetSensorInfra(i, sensor_infra[i]);

		VFH vfh(&robot, clientID,objectivo);
		vfh.Run();
		simxFinish(clientID);
	}

}


