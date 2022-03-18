# NIDAQ-ATI-ForceSensor

A package for reading and manipulating data from ATI force sensors using NI Daq Cards.
It Mainly contains code that read voltages from the NI daq card, and convert them to forces using the
`.CAL` file provided by ATI for the force/torque sensor.

The `src` folder contains the C++ libraries of NIDAQmx, which are usually present in the following directory if you 
have already installed NI drivers in your system:
`C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\include\NIDAQmx.h`
Never the less, we have included the headers just in case.

The current implementation only supports **Windows** platform. The class contains a header and a source file. 
The `.CAL` file inside `cal` directory is a sample ATI force/tprque sensor calibration file.

The `array_utils.h` header contains some useful functions for playing with dynamically allocated arrays,
matrices, etc. in the form of pointers. Go ahead and use them in your own programs, they come in handy.

The maximum (saturation) voltage is provided by the user, and is an input to the constructor.

The maximum (saturation) force/torque values are extracted from the `.CAL` file when parsing it. This file contains 
not only maximum force/torque values, but also the gain matrix used for converting voltages to force/torque values.

The ATI sensors have 6 gauges, therefore 6 voltage values, which are converted to 3 force values and 3 torque
values in the x, y, and z axes, respectively, concatenated to a 6x1 vector. The header `atisensors.h` contains
proper documentation for the functions and variables used in the code.

The audio fie `urgent.wav` is an alarm sound which will be played if voltage or force/torque saturation occurs.
This comes in handy especially in e.g. robotics/mechatronics and haptics applications where large force/torque values 
can occur, and potentially damage the sensors.

### Usage Example:

```C++
#include "atisensors.h"
#include <iostream>

using namespace std;

int main(){
    const char* cal_file = "cal/FT00001.cal"; // Provided by ATI
    const char* device_pins = "dev2/ai16:21"; // Index of pins on the NI Daq card where the sensor is installed
    double sampling_freq = 2000.0;            // Sampling frequency of the Daq card (Hz)
    double max_voltage = 10.0;                // Maximum (saturation) measureable voltage of the sensors (volts)
    double safety_factor = 0.9;               // Safety factor for detecting force/torque saturation
    size_t moving_ave_size = 5;               // Size of moving average for reading slightly smoother data
    size_t bias_len = 2500;                   // Length of data collection for initial biasing (calibration)
    
    atisensors ftsensor(cal_file, device_pins, sampling_freq, bias_len, moving_ave_size, max_voltage, safety_factor);
    ftsensor.run();

    // Wait for bias to be completed
    Sleep(10000);

    double forces[10];
    for(int i=0; i<10; i++) forces[i] = 0.0;
    for(int i=0; i<10; i++){
        Sleep(1000);
        ftsensor.getForces(forces);
    }
    cout << "Force measurements are:" << endl;
    for(int i=0; i<10; i++) cout << forces[i] << endl;

    return 0;
}
```

