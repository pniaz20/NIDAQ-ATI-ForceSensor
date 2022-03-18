#pragma once

#include <process.h>
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <sstream>
#include <array>
#include "array_utils.h"
#pragma comment(lib, "winmm.lib")

// NI-DAQmx headers are usually located in the following directory in Windows machines:
//#include "C:\\Program Files (x86)\\National Instruments\\Shared\\ExternalCompilerSupport\\C\\include\\NIDAQmx.h"

// In this implementation we have included them as a part of the repo.
#include "src\\NI\\ExternalCompilerSupport\\C\\include\\NIDAQmx.h"

using namespace std;
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else


class atisensors
{
public:
	atisensors();
	/**
	 * @brief Construct a new atisensors object
	 * 
	 * @param calfile: Calibration (.CAL) file, provided by ATI
	 * @param device_name_and_pins: String containing device and channel addresses, as in "dev2/ai16:21"
	 * @param samplingFreq: Daq card sampling frequency (Hz) 
	 * @param bias_length: Data collection window length for initial calibration (biasing)
	 * @param movingAverageWindowSize: Width of moving average for smoother results
	 * @param max_voltage: Maximum voltage (saturation voltage) measureable by the force sensors
	 * @param safety_factor: Safety factor for detecting force/torque saturation
	 */
	atisensors(const char* calfile, const char* device_name_and_pins, double samplingFreq, 
		size_t bias_length, size_t movingAverageWindowSize, double max_voltage, double safety_factor);
	~atisensors();
	/**
	 * @brief Initialize atisensors parameters and get it ready to start running
	 * 
	 * @param calFile: Calibration (.CAL) file provided by the manufacturer (ATI)
	 * @param device_name_and_pins: String containing device name and channel addresses, like "dev2/ai16:21"
	 * @param samplingFreq: Daq card sampling frequency (Hz)
	 */
	void initialize(const char* calFile, const char* device_name_and_pins, double samplingFreq);
	/**
	 * @brief Set the main parameters of the atisensors object so it can be initialized.
	 * 
	 * @param safety_factor: Safety factor for detecting force/torque saturation
	 * @param bias_length: Data collection window length for initial calibration (biasing)
	 * @param moving_average_size: Width of moving average for smoother results
	 * @param max_voltage: Maximum voltage (saturation voltage) measureable by the force sensors
	 */
	void setParams(double safety_factor, size_t bias_length, size_t moving_average_size, double max_voltage);
	/**
	 * @brief Get the latest measured forces
	 * 
	 * @param vec: Array in which to store the measurements
	 */
	void getForces(double* vec) const;
	/**
	 * @brief Get the latest measured voltages
	 * 
	 * @param vec: Array in which to store the measurements
	 */
	void getVoltages(double* vec) const;
	/**
	 * @brief Get whether or not there is a force/torque saturation based on the latest measurements
	 * 
	 * @return true if there is a gauge saturation
	 * @return false if there is no gauge saturation
	 */
	bool getForceSaturation() const;
	/**
	 * @brief Get whether or not there is a voltage saturation based on the latest measurements
	 * 
	 * @return true if there is a voltage saturation in the gauge measurements
	 * @return false if there is no voltage gauge saturation
	 */
	bool getVoltageSaturation() const;
	/**
	 * @brief Run the sensors. Note that the atisensor object must be initialized before being run,
	 * which does happen when using the custom constructor, but does not happen when using the default constructor.
	 * If the default constructor is used, the setParams() and initialize() functions need to be called before this
	 * function.
	 */
	void run();

private:
	
	static double safetyFactor;            // Safety factor for force saturation check
	static size_t biasLength;              // Number of time steps for initial biasing (calibration)
	static size_t movingforcewindowsize;   // Moving average window size
	static double MAX_VOLTAGE;             // Maximum voltage (saturation voltage) measureable by the sensors
	bool isInitialized;                    // Is never true until initialize() is executed.
	static CRITICAL_SECTION cs1;                    // For reading and storing calculated data
	static CRITICAL_SECTION sensorReadingSection;	// For reading data directly from the sensors
	static const char* deviceNameAndPins;			// String containing NI daq card device name and channel addresses
	static const char* cal_file;					// Path to the calibration fikle
	static size_t thisTotRun;						// Total number of runs in the current loop
	static size_t currentIndex;						// Current loop index in reading the data
	static size_t totRuns;							// Total number of reading runs
	static size_t processIndex;						// Total process index after biasing
	static bool biasIsReached;						// Whether enough data is collected for biasing
	static bool saturationVoltageCheck;				// Whether there is a saturation in voltages
	static bool force_saturation_check;				// Whether there is a saturation in force/torques
	static bool biasIsCompleted;					// Whether the initial biasing (calibration) has been done
	static bool endVoltageThread;					// Whether the voltage reading thread is over and needs to exit
	static double force_sampling_frequency;			// Daq card sampling frequency
	static double* vTemp;							// Temporary unbiased voltages
	static double* vBiasTemp;						// Temporary biased voltages
	static double* forceTemp;						// Temporary biased force vector
	static double* forceUnbiased;					// Unbiased force vector
	static double* FTLims; 							// Maximum/saturation force/torque values
	static double* averageLastForce;				// Result of the moving average of calculated forces
	static double* voltages;						// Voltages directly read form the sensor
	static double* voltages_;						// Voltages directly copied from the sensor measurements
	static double* forceSensorWindowSum;      		// Used for summing and averaging operations in the moving average
	static double* forceSensorCurrentVals;			// Used for adding the current measurements to the moving average
	static double* lastVoltages;					// Last voltages measured from the sensor
	static double* SampleBias;						// Data collected for the initial biasing (calibration)
	static double* FT1;								// Force vector stored after conversion
	static double** forceSensorWindow;				// Moving buffer of forces for moving average
	static double** biasVoltages;					// Bias voltages collected for biasing
	static double** GainMatrix; 					// Gain matrix for converting voltages to forces

	// Process the voltages acquired from the sensors and get force values
	static void processVoltages();

	// This thread starts reading voltages form the sensor
	static void startReadingVoltages(void* pParams); 

	// Check to see if there is a gauge saturation with the current measurements
	static bool checkForceTorqueSaturation(double* FTArr, double* FTLims); 

	// Parse the .CAL file and ewxtract calibration gain matrix and force/torque saturation values
	static void ReadCalFile(const char* filename);

	// Convert the measured voltages to force values using the gain matrix extracted from the .CAL file
	static void convertToForces(double* vbias, double* v, double* forceCurVector);

	// A function that is called back once every N system time steps, to read the data from the daq card
	static int32 CVICALLBACK EveryNCallback(TaskHandle taskHandle, int32 everyNsamplesEventType, 
        uInt32 nSamples, void* callbackData);

	// A function that is called once the program is done reading data from the sensors, and needs to clsoe the daq.
	static int32 CVICALLBACK DoneCallback(TaskHandle taskHandle, int32 status, void* callbackData);

	// The thread that listens to see if there is any voltage/force saturation, and plays an alarm sound if so.
	static void dummyThread(void* pParams);
	
	// Smoothens measured force data slightly
	static void movingAverage(size_t totalIndex, double* currentForces); 
};





