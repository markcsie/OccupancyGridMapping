#include "GridMapping.h"

GridMapping::GridMapping(const double &l0, const double &locc, const double &lfree, const double &alpha, const double &beta, const double &Zmax, const double &Zmin, const unsigned char &sensorType) {
    this -> l0 = l0;
    this -> locc = locc;
    this -> lfree = lfree;
    this -> alpha = alpha;
    this -> beta = beta * CV_PI / 180;
    this -> Zmax = Zmax;
    this -> Zmin = Zmin;
    this -> sensorType = sensorType;

    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            l[x][y] = l0;
        }
    }
    gridMapImage = cvCreateImage(cvSize(mapWidth / gridWidth, mapHeight/ gridHeight), IPL_DEPTH_8U, 1);
}

/** Occupancy Grid Mapping Algorithm, please refer to textbook "Probabilistic Robotics" */
void GridMapping::updateGridMap(const double &robotX, const double &robotY, const double &robotTheta, const double sensorData[]) {

    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            double xi, yi;
            gridToXY(x, y, xi, yi);
            if (sqrt(pow(xi - robotX, 2) + pow(yi - robotY, 2)) <= Zmax) {
                l[x][y] = l[x][y] + inverseSensorModel(robotX, robotY, robotTheta, xi, yi, sensorData) - l0;
            }
        }
    }
}
double GridMapping::inverseSensorModel(const double &x, const double &y, const double &theta, const double &xi, const double &yi, const double sensorData[]) {

    double r = sqrt(pow(xi - x, 2) + pow(yi - y, 2));
    double phi = atan2(yi - y, xi - x) - theta;


    double Zk;
    double thetaK;

    double sensorTheta;
    double minDelta = -1;

    switch (sensorType) {
    case 0:
        for (int i = 0; i < 361; i++) {
            sensorTheta = (-90 + i * 0.5) * (CV_PI / 180);
            if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
                Zk = sensorData[i];
                thetaK = sensorTheta;
                minDelta = fabs(phi - sensorTheta);
            }
        }
        break;
    case 1:
        /** -90 -37.5 -22.5 -7.5 7.5 22.5 37.5 90*/

        for (int i = 0; i < 8; i++) {
            if (i == 0) {
                sensorTheta = -90 * (CV_PI / 180);
            }
            else if (i == 1) {
                sensorTheta = -37.5 * (CV_PI / 180);
            }
            else if (i == 6) {
                sensorTheta = 37.5 * (CV_PI / 180);
            }
            else if (i == 7) {
                sensorTheta = 90 * (CV_PI / 180);
            }
            else {
                sensorTheta = (-37.5 + (i - 1) * 15) * (CV_PI / 180);
            }

            if (fabs(phi - sensorTheta) < minDelta || minDelta == -1) {
                Zk = sensorData[i];
                thetaK = sensorTheta;
                minDelta = fabs(phi - sensorTheta);            
            }
        }

        break;
    default:
        cout << "Unknown Sensor Type " << endl;
    }

    if (r > min((double)Zmax, Zk + alpha / 2) || fabs(phi - thetaK) > beta / 2 || Zk > Zmax || Zk < Zmin) {
        return l0;
    }
    else if (Zk < Zmax && fabs(r - Zk) < alpha / 2) {
        return locc;
    }
    else if (r <= Zk) {
        return lfree;
    }
}

void GridMapping::gridToXY(const int &x, const int &y, double &xi, double &yi) {
    xi = x * gridWidth + gridWidth / 2 - robotXOffset;
    yi = -(y * gridHeight + gridHeight / 2) + robotYOffset;
}

void GridMapping::showGridMap(const string &windowName) {
    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            double p = 1 - 1 / (1 + exp(l[x][y]));
            double grayValue = (double)((1 - p) * 255);
            CvScalar pixel = cvRealScalar(grayValue);
            cvSet2D(gridMapImage, y, x, pixel);
        }
    }

    cvNamedWindow(windowName.c_str(), 0);
    cvShowImage(windowName.c_str(), gridMapImage);

}

void GridMapping::saveGridMap(const string &fileName) {
    cvSaveImage(fileName.c_str(), gridMapImage);
}

