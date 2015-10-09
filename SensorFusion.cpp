#include "SensorFusion.h"

SensorFusion::SensorFusion(const GridMapping *gridMapping1, const GridMapping *gridMapping2) {
    gridWidth = gridMapping1 -> gridWidth;
    gridHeight = gridMapping1 -> gridHeight;
    mapWidth = gridMapping1 -> mapWidth;
    mapHeight = gridMapping1 -> mapHeight;

    /** 2-dimensional static-size array */
    p = (double**)malloc(sizeof(double*) * mapWidth / gridWidth);
    for(int x = 0; x < mapWidth / gridWidth; x++) {
        p[x] = (double*)malloc(sizeof(double) * mapHeight / gridHeight);
    }

    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            double p1 = 1 - 1 / (1 + exp(gridMapping1 -> l[x][y]));
            double p2 = 1 - 1 / (1 + exp(gridMapping2 -> l[x][y]));
            p[x][y] = 1 - (1 - p1) * (1 - p2);
        }
    }
    gridMapImage = cvCreateImage(cvSize(mapWidth / gridWidth, mapHeight/ gridHeight), IPL_DEPTH_8U, 1);
}

void SensorFusion::showGridMap(const string &windowName) {
    for (int x = 0; x < mapWidth / gridWidth; x++) {
        for (int y = 0; y < mapHeight / gridHeight; y++) {
            double grayValue = (double)((1 - p[x][y]) * 255);
            CvScalar pixel = cvRealScalar(grayValue);
            cvSet2D(gridMapImage, y, x, pixel);
        }
    }

    cvNamedWindow(windowName.c_str(), 0);
    cvShowImage(windowName.c_str(), gridMapImage);

}

void SensorFusion::saveGridMap(const string &fileName) {
    cvSaveImage(fileName.c_str(), gridMapImage);
}
