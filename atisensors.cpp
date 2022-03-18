#include "atisensors.h"
#include "array_utils.h"
#include <assert.h>

using namespace std;


atisensors::atisensors():isInitialized(false) { setParams(0.9, 2500, 5, 10.0); }


atisensors::atisensors(
	const char* calfile, 
	const char* device_name_and_pins,
	double samplingFreq, 
	size_t bias_length, 
	size_t movingAverageWindowSize, 
	double max_voltage, 
	double safety_factor) : isInitialized(false){
	setParams(safety_factor, bias_length, movingAverageWindowSize, max_voltage);
	initialize(calfile, device_name_and_pins, samplingFreq);
}

atisensors::~atisensors(){
    delete[] vTemp;
	delete[] vBiasTemp;
	delete[] forceTemp;
	delete[] forceUnbiased;
	delete[] GainMatrix; 
	delete[] FTLims; 
	delete[] averageLastForce;
	delete[] voltages;
	delete[] voltages_;
	delete[] forceSensorWindowSum;      
	delete[] forceSensorCurrentVals;
	delete[] lastVoltages;
	delete[] SampleBias;
	delete[] FT1;
	delete[] forceSensorWindow;
	delete[] biasVoltages;
};


void atisensors::setParams(double safety_factor, size_t bias_length, size_t moving_average_size, double max_voltage){
	safetyFactor = safety_factor;
	biasLength = bias_length;
	movingforcewindowsize = moving_average_size;
	MAX_VOLTAGE = max_voltage;
}


void atisensors::initialize(const char* calFile, const char* device_name_and_pins, double samplingFreq)
{
	cal_file = calFile;
	ReadCalFile(calFile);
    vTemp = allocateVector<double>(6);
    vBiasTemp = allocateVector<double>(6);
    forceTemp = allocateVector<double>(6);
	vTemp = allocateVector<double>(6);
	vBiasTemp = allocateVector<double>(6);
	forceTemp = allocateVector<double>(6);
	forceUnbiased = allocateVector<double>(6);
	FTLims = allocateVector<double>(6); 
	averageLastForce = allocateVector<double>(6);
	voltages = allocateVector<double>(6);
	voltages_ = allocateVector<double>(6);
	lastVoltages = allocateVector<double>(6);
	SampleBias = allocateVector<double>(6);
	FT1 = allocateVector<double>(6);
	forceSensorWindowSum = allocateVector<double>(6);
	forceSensorCurrentVals = allocateVector<double>(6);
	GainMatrix = allocateMatrix<double>(6, 6); 
	forceSensorWindow = allocateMatrix<double>(6, movingforcewindowsize);
	biasVoltages = allocateMatrix<double>(biasLength, 6);
	totRuns = 0;
	thisTotRun = 0;
	processIndex = 0;
	endVoltageThread = false;
	biasIsCompleted = false;
	saturationVoltageCheck = false;
	force_saturation_check = false;
	biasIsReached = false;
	deviceNameAndPins = device_name_and_pins;
	force_sampling_frequency = samplingFreq;
	InitializeCriticalSection(&cs1);
	InitializeCriticalSection(&sensorReadingSection);
	isInitialized = true;
}



void atisensors::ReadCalFile(const char* filename) {
    // calibration files are .cal files
	ifstream calfile(filename);
    double** calMat = allocateMatrix<double>(6, 6);
    double* maxVec = allocateVector<double>(6);
	if (calfile.is_open()) {
		std::string line;
		std::string delimiter = "\"";
		int row = 0;
		while (std::getline(calfile, line)) { // In each line
			if (line.find("<UserAxis") != std::string::npos) { // Get Only User Axis
				size_t pos = 0;
				int counter = 0;
				std::string token;
				// Splitting for quotation marks
				while ((pos = line.find(delimiter)) != std::string::npos) {
					token = line.substr(0, pos); // get the splitted string
					if (counter == 3) { // Row of calibration matrix
						vector <double> tempval;
						stringstream check1(token);
						string intermediate;
						string::size_type sz;
						// Tokenizing w.r.t. space ' '
						while (getline(check1, intermediate, ' '))
						{
							if (intermediate != "") {
								tempval.push_back(stod(intermediate, &sz));
							}
						}
						// Printing the token vector
						for (int i = 0; i < tempval.size(); i++) {
							calMat[row][i] = tempval[i];
						}
					}
					else if (counter == 5) { // Max value
						string::size_type sz;
						maxVec[row] = stod(token, &sz);
					}
					line.erase(0, pos + delimiter.length());
					counter++;
				}
				row++;
			}
		}
		calfile.close();
	}
	for(int i=0; i < 6; i++){
		FTLims[i] = maxVec[i];
		for(int j=0; j < 6; j++) GainMatrix[i][j] = calMat[i][j];
	}
	delete[] calMat;
	delete[] maxVec;
}



void atisensors::run()
{
	assert(isInitialized);
	_beginthread(startReadingVoltages, 0, NULL);
	_beginthread(dummyThread, 0, NULL);
}


void atisensors::startReadingVoltages(void* pParams)
{
	if (!InitializeCriticalSectionAndSpinCount(&sensorReadingSection, 0x00000400))
	{
		endVoltageThread = true;
	}
	int32       error = 0;
	TaskHandle  taskHandle = 0;
	char        errBuff[2048] = { '\0' };
	/*********************************************/
	// DAQmx Configure Code
	/*********************************************/
	DAQmxErrChk(DAQmxCreateTask("", &taskHandle));
	cout << "force task handle" << taskHandle << endl;
	DAQmxErrChk(DAQmxCreateAIVoltageChan(taskHandle, deviceNameAndPins, "", 
        DAQmx_Val_Cfg_Default, -MAX_VOLTAGE, MAX_VOLTAGE, DAQmx_Val_Volts, NULL));
	DAQmxErrChk(DAQmxCfgSampClkTiming(taskHandle, "", force_sampling_frequency, 
        DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1));
	DAQmxErrChk(DAQmxRegisterEveryNSamplesEvent(taskHandle, DAQmx_Val_Acquired_Into_Buffer, 
        1, 0, EveryNCallback, NULL));
	DAQmxErrChk(DAQmxRegisterDoneEvent(taskHandle, 0, DoneCallback, NULL));
	/*********************************************/
	// DAQmx Start Code
	/*********************************************/
	DAQmxErrChk(DAQmxStartTask(taskHandle));

	// dummy lines to prevent function from ending.
	int tempVar = 0;
	while (!endVoltageThread)
	{
		tempVar = 0;
		Sleep(100);
	}
	printf("Threads ended\n");

Error:
	if (DAQmxFailed(error))
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
	if (taskHandle != 0) {
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
	}
	if (DAQmxFailed(error))
		printf("DAQmx Error: %s\n", errBuff);
	printf("End of program, press Enter key to quit\n");
}







int32 CVICALLBACK atisensors::EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, 
    uInt32 nSamples, void* callbackData)
{
	int32       error = 0;
	char        errBuff[2048] = { '\0' };
	static int  totalRead = 0;
	int32       read = 0;
	float64     data[6];

	/*********************************************/
	// DAQmx Read Code
	/*********************************************/
	DAQmxErrChk(DAQmxReadAnalogF64(taskHandle, 1, 10.0, DAQmx_Val_GroupByScanNumber, data, 6, &read, NULL));
	if (read > 0) {
		totalRead += read;
		EnterCriticalSection(&sensorReadingSection);
		totRuns++;
		for (int i = 0; i < 6; i++) voltages[i] = data[i];
		LeaveCriticalSection(&sensorReadingSection);
		if (totalRead == biasLength + 1 && !biasIsReached) biasIsReached = true;
		if (!biasIsReached)
			for (int i = 0; i < 6; i++) biasVoltages[totRuns - 1][i] = data[i];
	}
	processVoltages();

Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		/*********************************************/
		// DAQmx Stop Code
		/*********************************************/
		DAQmxStopTask(taskHandle);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
	}


	return 0;
}




int32 CVICALLBACK atisensors::DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData)
{
	int32   error = 0;
	char    errBuff[2048] = { '\0' };

	// Check to see if an error stopped the task.
	DAQmxErrChk(status);

Error:
	if (DAQmxFailed(error)) {
		DAQmxGetExtendedErrorInfo(errBuff, 2048);
		DAQmxClearTask(taskHandle);
		printf("DAQmx Error: %s\n", errBuff);
	}
	return 0;
}



void atisensors::processVoltages() 
{
	int temp = 0;

	if (!biasIsReached)
		temp++;
	else if (!biasIsCompleted)
	{
        double sumBias;
        int oldRunTime = 0;
        for (int i = 0; i < 6; i++)
        {
            sumBias = 0;
            for (int j = 0; j < biasLength; j++) sumBias = sumBias + biasVoltages[j][i];
            SampleBias[i] = sumBias / biasLength;
        }
        processIndex = 0;
        biasIsCompleted = true;
	}


	if (biasIsCompleted)
	{
		for (int i = 0; i < 6; i++)
		{
			processIndex++;
			EnterCriticalSection(&sensorReadingSection);
			voltages_[i] = voltages[i];
			thisTotRun = totRuns;
			LeaveCriticalSection(&sensorReadingSection);
			if (voltages_[i] > MAX_VOLTAGE || voltages_[i] < -MAX_VOLTAGE)
			{
				EnterCriticalSection(&cs1);
				saturationVoltageCheck = true;
				LeaveCriticalSection(&cs1);
			}
		        
		}
		convertToForces(SampleBias, voltages_, FT1);
		for (int i = 0; i < 6; i++) forceSensorCurrentVals[i] = FT1[i];
		movingAverage(thisTotRun - 1, forceSensorCurrentVals);

		EnterCriticalSection(&cs1);
		force_saturation_check = checkForceTorqueSaturation(forceUnbiased, FTLims);
		for (int i = 0; i < 6; i++)
		{
			averageLastForce[i] = forceSensorWindowSum[i];
			lastVoltages[i] = voltages_[i];
		}
		LeaveCriticalSection(&cs1);
	}

}



void atisensors::convertToForces(double* vbias, double* v, double* forceCurVector)
{
	double forceCurrent[6] = { 0.0,0.0,0.0,0.0,0.0,0.0 };
	for (int i = 0; i < 6; i++)
	{
		vTemp[i] = v[i];
		vBiasTemp[i] = vbias[i];
	}

	// conversion and matrix multiplication
    double* sub = allocateVector<double>(6);
    subtract<double>(sub, vTemp, vBiasTemp, 6);
    multiplyVector<double>(forceTemp, GainMatrix, sub, 6, 6);
    multiplyVector<double>(forceUnbiased, GainMatrix, vTemp, 6, 6);
	for (int i = 0; i < 6; i++) forceCurVector[i] = forceTemp[i];
}


void atisensors::movingAverage(size_t totalIndex, double* currentForces)
{
	currentIndex = totalIndex % movingforcewindowsize;
	for (int i = 0; i < 6; i++)
	{
		forceSensorWindowSum[i] = (movingforcewindowsize * forceSensorWindowSum[i] - 
            forceSensorWindow[i][currentIndex] + currentForces[i]) / movingforcewindowsize;
		forceSensorWindow[i][currentIndex] = currentForces[i];
	}
}



bool atisensors::checkForceTorqueSaturation(double* FTArr, double* FTLims) {
	for (int i = 0; i < 6; i++)
		if (abs(FTArr[i]) > safetyFactor * FTLims[i]) 
			return true;
	return false;
}



void atisensors::dummyThread(void* pParams)
{
	size_t tempVar = 0;
	const wchar_t* alert_filename = L"urgent.wav";
	while (true) {
		if (saturationVoltageCheck || force_saturation_check)
		{
			PlaySound(alert_filename, NULL, SND_FILENAME);
			Sleep(500);
		}
		tempVar++;
	}
}



void atisensors::getForces(double* vec) const {
	EnterCriticalSection(&cs1);
	for(int i=0; i < 6; i++) vec[i] = averageLastForce[i];
	LeaveCriticalSection(&cs1);
}


void atisensors::getVoltages(double* vec) const {
	EnterCriticalSection(&cs1);
	for(int i=0; i < 6; i++) vec[i] = lastVoltages[i];
	LeaveCriticalSection(&cs1);
}


bool atisensors::getForceSaturation() const {
	bool c = false;
	EnterCriticalSection(&cs1);
	c = force_saturation_check;
	LeaveCriticalSection(&cs1);
	return c;
}


bool atisensors::getVoltageSaturation() const {
	bool c = false;
	EnterCriticalSection(&cs1);
	c = saturationVoltageCheck;
	LeaveCriticalSection(&cs1);
	return c;
}


double* atisensors::averageLastForce;
double* atisensors::vTemp;
double* atisensors::vBiasTemp;
double* atisensors::forceTemp;
double* atisensors::forceUnbiased;
double* atisensors::FTLims;
double* atisensors::voltages;
double* atisensors::FT1;
double* atisensors::SampleBias;
double* atisensors::voltages_;
double* atisensors::lastVoltages;
double* atisensors::forceSensorWindowSum;
double* atisensors::forceSensorCurrentVals;
double** atisensors::GainMatrix;
double** atisensors::biasVoltages;
double** atisensors::forceSensorWindow;
const char* atisensors::deviceNameAndPins;
const char* atisensors::cal_file;
CRITICAL_SECTION atisensors::cs1;
CRITICAL_SECTION atisensors::sensorReadingSection;
size_t atisensors::totRuns;
size_t atisensors::thisTotRun;
size_t atisensors::currentIndex;
size_t atisensors::processIndex;
size_t atisensors::biasLength;           
size_t atisensors::movingforcewindowsize;
bool atisensors::biasIsCompleted;
bool atisensors::saturationVoltageCheck;
bool atisensors::force_saturation_check;
bool atisensors::biasIsReached;
bool atisensors::endVoltageThread;
double atisensors::safetyFactor;           
double atisensors::MAX_VOLTAGE; 
double atisensors::force_sampling_frequency;        