#include <iostream>
using namespace std;

#include <stdio.h>
#include <string.h>
#include <cv.h>
#include <highgui.h>

#include "GridMapping.h"
#include "SensorFusion.h"

int main (int argc, char **argv) {
    if (argc < 4) {
        cerr << "Input Error!!!";
        return -1;
    }

    /** Read Files */
    FILE *odomFile = fopen(argv[1], "r");
    FILE *laserFile = fopen(argv[2], "r");
    FILE *sonarFile = fopen(argv[3], "r");

    double timeStamp;

    double laserData[361];
    double sonarData[8];

    double robotX;
    double robotY;
    double robotTheta;

    GridMapping *laserGridMapping = new GridMapping(0, 2.2, -2.2, 200, 0.5, 80000, 0, 0);

    GridMapping *sonarGridMapping = new GridMapping(0, 0.4, -0.4, 200, 20, 5000, 170, 1);

    int frameCount = 1;

    while (fscanf (odomFile, "%lf %lf %lf %lf", &timeStamp, &robotX, &robotY, &robotTheta) != EOF) {
        cout << "Frame " << frameCount << endl;


        fscanf (laserFile, "%lf", &timeStamp);
        for (int i = 0; i < 361; i++) {
            fscanf (laserFile, "%lf", &laserData[i]);
        }
        laserGridMapping -> updateGridMap(robotX, robotY, (robotTheta / 10) * (CV_PI / 180), laserData);

        fscanf (sonarFile, "%lf", &timeStamp);
        for (int i = 0; i < 8; i++) {
            fscanf (sonarFile, "%lf", &sonarData[i]);
        }
        sonarGridMapping -> updateGridMap(robotX, robotY, (robotTheta / 10) * (CV_PI / 180), sonarData);

        frameCount++;
    }

    SensorFusion *sensorFusion = new SensorFusion(laserGridMapping, sonarGridMapping);

    laserGridMapping -> showGridMap("Laser Grid Map");
    sonarGridMapping -> showGridMap("Sonar Grid Map");
    sensorFusion -> showGridMap("Sensor Fusion Map");


    if (cvWaitKey(0) == 'q') {
        cout << "============End============" << endl;
    }

    laserGridMapping -> saveGridMap("LaserGridMap.jpg");
    sonarGridMapping -> saveGridMap("SonarGridMap.jpg");
    sensorFusion -> saveGridMap("SensorFusionGridMap.jpg");

    fclose(odomFile);
    fclose(laserFile);
    fclose(sonarFile);

    return 0;
}

