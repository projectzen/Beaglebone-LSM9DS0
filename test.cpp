#include "SFE_LSM9DS0.h"
#include <iostream>
#include <unistd.h>

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

using namespace std;

LSM9DS0 dof(1, LSM9DS0_G, LSM9DS0_XM);

int main() {
  int status = dof.begin();
  cout << "Status: " << status << endl;
	while (1) {
		// To read from the accelerometer, you must first call the
		// readAccel() function. When this exits, it'll update the
		// ax, ay, and az variables with the most current data.
		dof.readAccel();

		// Now we can use the ax, ay, and az variables as we please.
		// Either print them as raw ADC values, or calculated in g's.
		cout << "A: ";
		// If you want to print calculated values, you can use the
		// calcAccel helper function to convert a raw ADC value to
		// g's. Give the function the value that you want to convert.
		cout << dof.calcAccel(dof.ax);
		cout << ", ";
		cout << dof.calcAccel(dof.ay);
		cout << ", ";
		cout << dof.calcAccel(dof.az) << endl;

		dof.readGyro();

		// Now we can use the gx, gy, and gz variables as we please.
		// Either print them as raw ADC values, or calculated in DPS.
		cout << "G: ";
		cout << dof.calcGyro(dof.gx);
		cout << ", ";
		cout << dof.calcGyro(dof.gy);
		cout << ", ";
		cout << dof.calcGyro(dof.gz) << endl;

		dof.readMag();
		
		// Now we can use the mx, my, and mz variables as we please.
		// Either print them as raw ADC values, or calculated in Gauss.
		cout << "M: ";
		// If you want to print calculated values, you can use the
		// calcMag helper function to convert a raw ADC value to
		// Gauss. Give the function the value that you want to convert.
		cout << dof.mx;
		cout << ", ";
		cout << dof.my;
		cout << ", ";
		cout << dof.mz << endl;

		dof.readTemp();
		cout << "T: " << (21.0 + (float)dof.temperature/8.) << endl;
		sleep(1);
	}
}
