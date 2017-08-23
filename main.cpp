#include <iostream>
#include "include/Tracking.h"
#include "include/Camera.h"

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

int main() {

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;

    string left = "/home/t/Documents/ORB_SLAM2-master/Examples/Stereo/Dissertation/03";
    LoadImages(left,vstrImageLeft,vstrImageRight,vTimestamps);

    cv::Mat imLeft, imRight, imLeftRect, imRightRect,K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r,Q;
    cv::Size size, newsize;
    cv::Mat R,t;

//    int rows_l = 240;
//    int cols_l = 320;
//    int rows_r = 240;
//    int cols_r = 320;
//    D_l = (cv::Mat_<double>(1,5) << -0.333236, 0.925076, 0.003847, 0.0000916, 0.0);
//    K_l = (cv::Mat_<double>(3,3) << 381.914307, 0.0, 168.108963, 0.0,383.797882, 126.979446, 0.0, 0.0, 1.0);
//    R_l = cv::Mat::eye(3,3,CV_64F);
//    P_l = (cv::Mat_<double>(3,4) << 435.2046959714599, 0, 367.4517211914062, 0,  0, 435.2046959714599, 252.2008514404297, 0,  0, 0, 1, 0);
//    D_r = (cv::Mat_<double>(1,5) << -0.329342, 0.699034, -0.004927,0.008194, 0.0);
//    K_r = (cv::Mat_<double>(3,3) << 381.670013, 0.0, 129.929291, 0.0, 382.582397, 120.092186, 0.0, 0.0, 1);
//    R_r = (cv::Mat_<double>(3,3) << 0.9999633526194376, -0.003625811871560086, 0.007755443660172947, 0.003680398547259526, 0.9999684752771629, -0.007035845251224894, -0.007729688520722713, 0.007064130529506649, 0.999945173484644);
//    P_r = (cv::Mat_<double>(3,4) << 435.2046959714599, 0, 367.4517211914062, 0,  0, 435.2046959714599, 252.2008514404297, 0,  0, 0, 1, 0);
//    R = (cv::Mat_<double>(3,3) << 0.999906,0.006813,-0.011930, -0.006722,0.999948, 0.007680,0.011981,-0.007599,0.999899);
//    t = (cv::Mat_<double>(3,1) << 5.382236, 0.067659, -0.039156);


    int rows_l = 1241;
    int cols_l = 376;
    int rows_r = 1241;
    int cols_r = 376;

    size = cv::Size(cols_l,rows_l);
    D_l = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0.0);
    K_l = (cv::Mat_<double>(3,3) << 721.5377, 0.0, 609.5593, 0.0,721.5377, 172.854, 0.0, 0.0, 1.0);

    D_r = (cv::Mat_<double>(1,5) << 0, 0, 0, 0, 0.0);
    K_r = (cv::Mat_<double>(3,3) << 721.5377, 0.0, 609.5593, 0.0,721.5377, 172.854, 0.0, 0.0, 1);

//    cv::Mat M1l,M2l,M1r,M2r;
//    cv::stereoRectify(K_l, D_l, K_r, D_r, cv::Size(1241,376), R, t, R_l, R_r, P_l, P_r, Q, cv::CALIB_ZERO_DISPARITY,-1,newsize);
//
//    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
//    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    Dissertation::Camera camera = Dissertation::Camera(1241,376,
                                                       721.5377,721.5377,
                                                       609.5593,172.854,
                                                       0.0, 0.0, 0.0, 0.0, 0.0,
                                                       40, 387.5744);

    Dissertation::Tracking tracking = Dissertation::Tracking(&camera);

    for(int i = 0 ; i < vstrImageLeft.size();i++)
    {
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[i],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[i],CV_LOAD_IMAGE_UNCHANGED);

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[i]) << endl;
            return 1;
        }

        tracking.Track_Stereo(imLeft,imRight,vTimestamps[i]);
    }

    return 0;
}