drawLRFScan(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist)
{
    Vec2i imageHalfSize = Vec2i(display.cols/2, display.rows/2);
    int nRangeSize = (int)laserScanXY.size();

    for(int i=0; i<nRangeSize; i++) {
        int x = imageHalfSize[0] + cvRound((laserScanXY[i][0]/dMaxDist)*imageHalfSize[0]);
        int y = imageHalfSize[1] + cvRound((laserScanXY[i][1]/dMaxDist)*imageHalfSize[1]);

        if(x >= 0 && x < display.cols && y >= 0 && y < display.rows) {
            display.at<Vec3b>(y, x) = Vec3b(255, 255, 0);
        }
    }
}

