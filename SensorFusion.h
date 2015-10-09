#ifndef SENSORFUSION_H
#define SENSORFUSION_H

/** Opencv includes */
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "GridMapping.h"

class SensorFusion {
public:
    SensorFusion(const GridMapping *gridMapping1, const GridMapping *gridMapping2);
    void showGridMap(const string &windowName);
    void saveGridMap(const string &fileName);

private:
    IplImage *gridMapImage;

    /** Grid Size (mm) */
    int gridWidth; // 200 for data1, 100 for data2, 100 for data3, 100 for data4, 100 for data5,
    int gridHeight; // 200 for data1, 100 for data2, 100 for data3, 100 for data4, 100 for data5,

    /** Map Size (mm) */
    int mapWidth; // 35000 for data1, 30000 for data2, 45000 for data3, 15000 for data4, 30000 for data5
    int mapHeight; // 60000 for data1, 30000 for data2, 25000 for data3, 50000 for data4, 15000 for data5

    double **p;

};

#endif // SENSORFUSION_H
