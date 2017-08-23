//
// Created by t on 8/22/17.
//

#include "QuasiDenseStereo.h"

namespace Dissertation {
    inline bool CheckBorder(Match m, int bx, int by, int w, int h) {
        if (m.p0.x < bx || m.p0.x > w - bx || m.p0.y < by || m.p0.y > h - by || m.p1.x < bx || m.p1.x > w - bx ||
            m.p1.y < by || m.p1.y > h - by) {
            return false;
        }
        return true;
    }

// Used for sorting
    bool MatchCompare(Match a, Match b) {
        if (a.corr <= b.corr)return true;
        return false;
    }

    QuasiDenseStereo::QuasiDenseStereo(void) {
        curr = 0;
        past = 2;
    }

    QuasiDenseStereo::~QuasiDenseStereo(void) {}

    void QuasiDenseStereo::initialize(cv::Size) {}

    void QuasiDenseStereo::process(cv::Mat imgL, cv::Mat imgR) {
        mRefTextureDesc = new int[imgL.cols * imgL.rows];
        mDstTextureDesc = new int[imgL.cols * imgL.rows];
        mImageL = (imgL).clone();
        mImageR = (imgR).clone();

        sparseMatching(&mImageL, &mImageR);
        quasiDenseMatching(&mImageL, &mImageR, &mPoints2DL, &mPoints2DR, &mStatusL);
    }

    void QuasiDenseStereo::sparseMatching(cv::Mat *imgL, cv::Mat *imgR) {
        // some parameters
        cv::Size templateSize = cv::Size(5, 5);
        int numPyrLevels = 3;
        double qualityThreshold = 0.01;
        double minSeparationDistance = 10;

        cv::goodFeaturesToTrack(*imgL, mPoints2DL, 100, qualityThreshold, minSeparationDistance, cv::noArray(), 3,
                                false, 0.04);

        std::vector<cv::Mat> temp;
        std::vector<float> err;
        cv::calcOpticalFlowPyrLK((*imgL), (*imgR), mPoints2DL, mPoints2DR,
                                 mStatusL, err, templateSize, numPyrLevels,
                                 cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 30, 0.00003), 0, 1e-4);
    }

    void QuasiDenseStereo::buildTextureDescriptor(cv::Mat *img, int *descriptor) {
        int x, y, cb;
        int nb = img->channels(), w = img->cols, h = img->rows;
        int s = int(img->step);

        float a, b, c, d, sb = 1.f / (float) nb;

        memset(descriptor, 0, sizeof(int) * w * h);

        for (y = 1; y < h - 1; y++) {
            for (x = 1; x < w - 1; x++) {
                //current pixel
                cv::Vec3b pixel = img->at<cv::Vec3b>(y, x);
                //pixel = &imgData[y*s+x*nb];

                //difference
                a = 0;
                b = 0;
                c = 0;
                d = 0;
                for (cb = 0; cb < nb; cb++) {
                    a += (float) abs(pixel[cb] - pixel[cb - nb]);
                    b += (float) abs(pixel[cb] - pixel[cb + nb]);
                    c += (float) abs(pixel[cb] - pixel[cb - s]);
                    d += (float) abs(pixel[cb] - pixel[cb + s]);
                }

                a *= sb;
                b *= sb;
                c *= sb;
                d *= sb;

                int val = max(a, max(b, max(c, d)));

                // clamp
                if (val > 255) val = 255;
                descriptor[y * w + x] = val;
                // just so we know the data ranges
            }
        }
    }

    cv::Point QuasiDenseStereo::getMatch(int x, int y) {
        return (mRefMap)[y * mImageL.cols + x];
    }

    float QuasiDenseStereo::iZNCC_c1(cv::Point p0, cv::Point p1, int wx, int wy) {
        int x, y, otl, otr, obl, obr, wx1, wy1;
        //Means and standard deviation
        float m0 = 0.f, m1 = 0.f;
        double s0 = 0.f, s1 = 0.f;
        //Window area
        float wa = (float) (2 * wy + 1) * (2 * wx + 1);
        float zncc = 0.f;
        //wy, wx is the radius of the window，
        int boy0 = (p0.y - wy) * Step + (p0.x - wx);
        int boy1 = (p1.y - wy) * Step + (p1.x - wx);

        int oy0 = boy0, oy1 = boy1;
        int ox0 = 0, ox1 = 0;

        // offsets for corners top-left, top-right, bottom-left, bottom-right
        wx1 = wx + 1;
        wy1 = wy + 1;

        // offsets for left image
        otl = (p0.y - wy) * Steps + (p0.x - wx);
        otr = (p0.y - wy) * Steps + (p0.x + wx1);
        obl = (p0.y + wy1) * Steps + (p0.x - wx);
        obr = (p0.y + wy1) * Steps + (p0.x + wx1);

        m0 = (float) ((sum0[obr] + sum0[otl]) - (sum0[obl] + sum0[otr]));
        s0 = (double) (ssum0[obr] + ssum0[otl]) - (ssum0[obl] + ssum0[otr]);

        // offsets for right image
        otl = (p1.y - wy) * Steps + (p1.x - wx);
        otr = (p1.y - wy) * Steps + (p1.x + wx1);
        obl = (p1.y + wy1) * Steps + (p1.x - wx);
        obr = (p1.y + wy1) * Steps + (p1.x + wx1);

        // sum and squares sum for right window
        m1 = (float) ((sum1[obr] + sum1[otl]) - (sum1[obl] + sum1[otr]));
        s1 = (double) (ssum1[obr] + ssum1[otl]) - (ssum1[obl] + ssum1[otr]);


        // window means
        m0 /= wa;
        m1 /= wa;
        // standard deviations

        s0 = sqrt((s0 - wa * m0 * m0));
        s1 = sqrt((s1 - wa * m1 * m1));

        zncc = 0;
        for (y = -wy; y <= wy; y++, oy1 += Step, oy0 += Step) {
            ox0 = 0, ox1 = 0;
            unsigned char *line0 = &data0[oy0];
            unsigned char *line1 = &data1[oy1];
            for (x = -wx; x <= wx; x++) {
                zncc += (float) line0[ox0++] * (float) line1[ox1++];
            }
        }

        // the final result
        zncc = (zncc - wa * m0 * m1) / (s0 * s1);
        return zncc;
    }

    void QuasiDenseStereo::quasiDenseMatching(cv::Mat *imgL, cv::Mat *imgR, vector<cv::Point2f> *pointsL,
                                              vector<cv::Point2f> *pointsR, vector<uchar> *featureStatus) {
        int x, y, i, wx, wy, NumberOfMatchesFound = 0;

        // reset memory of maps
        mRefMap = *new vector<cv::Point>[imgL->cols * imgL->rows];
        mMtcMap = *new vector<cv::Point>[imgL->cols * imgL->rows];
        for (int i = 0; i < (imgL->cols * imgL->rows); i++) {
            mRefMap.push_back(cv::Point(0, 0));
            mMtcMap.push_back(cv::Point(0, 0));
        }

        // build texture homogeneity reference maps.
        buildTextureDescriptor(imgL, mRefTextureDesc);
        buildTextureDescriptor(imgR, mDstTextureDesc);

        cv::Mat img2(320, 288, CV_8UC3);

        for (int i = 0; i < 360 * 288; i++) {
            int y = floor(i / 360);
            x = i - (y * 360);
            //img2.at<>(y,x) = mRefTextureDesc[i];
            //printf("%d\n",img2.at<int>(y,x));
        }

        // generate the intergal images for fast variable window correlation calculations
        cv::integral((*imgL), mIntegralImageL, mIntegralImageSqL);
        cv::integral((*imgR), mIntegralImageR, mIntegralImageSqR);

        data0 = (*imgL).data;
        data1 = (*imgR).data;

        sum0 = (int *) mIntegralImageL.data;
        sum1 = (int *) mIntegralImageR.data;
        ssum0 = (double *) mIntegralImageSqL.data;
        ssum1 = (double *) mIntegralImageSqR.data;


        Step = (int) imgL->step;
        Steps = mIntegralImageL.cols;

        // init these vars
        costWidth = imgL->cols;
        costHeight = imgL->rows;
        costDepth = 15;

        // Seed list
        std::priority_queue<Match, vector<Match>, less<Match> > Seed;

        // Build a list of seeds from the starting features
        for (unsigned int i = 0; i < mStatusL.size(); i++) {
            if (int(mStatusL[i]) == 1) {
                // Calculate correlation and store match in Seeds.
                Match m;
                m.p0 = (*pointsL)[i];
                m.p1 = (*pointsR)[i];

                // Check if too close to boundary.
                if (!CheckBorder(m, Param.BorderX, Param.BorderY, imgL->cols, imgL->rows))
                    continue;
                // Calculate the correlation threshold
                m.corr = iZNCC_c1(m.p0, m.p1, Param.WinSizeX, Param.WinSizeY);

                // Can we add it to the list
                if (m.corr > Param.Ct) {
                    // FIXME: Check if this is unique (or assume it is due to prior supression)
                    Seed.push(m);
                    mRefMap[m.p0.y * imgL->cols + m.p0.x] = m.p1;
                    mMtcMap[m.p1.y * imgL->cols + m.p1.x] = m.p0;
                }
            }
        }

        int total = 0;
        int counter = 0;
        // Do the propagation part
        while (!Seed.empty()) {
            priority_queue<Match, vector<Match>, less<Match> > Local;
            // Get the best seed at the moment
            Match m = Seed.top();
            Seed.pop();
            // Ignore the border
            if (!CheckBorder(m, Param.BorderX, Param.BorderY, imgL->cols, imgL->rows))continue;

            // For all neighbours in image 1
            for (y = -Param.N; y <= Param.N; y++) {
                for (x = -Param.N; x <= Param.N; x++) {
                    total += 1;
                    cv::Point p0 = cv::Point(m.p0.x + x, m.p0.y + y);
                    // Check if its unique in ref
                    if ((mRefMap)[p0.y * imgL->cols + p0.x].x != NO_MATCH)continue;

                    counter += 1;
                    // Check the texture descriptor for a boundary
                    if (mRefTextureDesc[p0.y * imgL->rows + p0.x] > Param.Tt)continue;


                    // For all candidate matches.
                    for (wy = -Param.Dg; wy <= Param.Dg; wy++) {
                        for (wx = -Param.Dg; wx <= Param.Dg; wx++) {
                            cv::Point p1 = cv::Point(m.p1.x + x + wx, m.p1.y + y + wy);

                            // Check if its unique in ref
                            if ((mMtcMap)[p1.y * imgL->cols + p1.x].x != NO_MATCH) continue;


                            // Check the texture descriptor for a boundary
                            if (mDstTextureDesc[p1.y * imgL->cols + p1.x] > Param.Tt) continue;

                            // Calculate ZNCC and store local match.
                            float corr = iZNCC_c1(p0, p1, Param.WinSizeX, Param.WinSizeY);

                            // push back if this is valid match
                            if (corr > Param.Ct) {
                                Match nm;
                                nm.p0 = p0;
                                nm.p1 = p1;
                                nm.corr = corr;
                                Local.push(nm);
                            }
                        }
                    }
                }
            }

            // Get seeds from the local
            while (!Local.empty()) {
                Match lm = Local.top();
                Local.pop();

                // Check if its unique in both ref and dst.
                if ((mRefMap)[lm.p0.y * imgL->cols + lm.p0.x].x != NO_MATCH) continue;
                if ((mMtcMap)[lm.p1.y * imgL->cols + lm.p1.x].x != NO_MATCH) continue;

                // Unique match
                (mRefMap)[lm.p0.y * imgL->cols + lm.p0.x] = lm.p1;
                (mMtcMap)[lm.p1.y * imgL->cols + lm.p1.x] = lm.p0;

                // Add to the seed list
                Seed.push(lm);
                NumberOfMatchesFound++;
            }
        }
    }

    pair<vector<cv::Point2f>, vector<cv::Point2f>> QuasiDenseStereo::AssignDispToImage(cv::Mat &output) {

        vector<float> temp = vector<float>(mImageL.rows * mImageR.cols, 0);
        float mindsp = 0;
        float maxdsp = mImageL.cols;
        int counter = 0;
        pair<vector<cv::Point2f>, vector<cv::Point2f>> features;
        for (int y = 0; y < mImageL.rows; y++) {
            for (int x = 0; x < (mImageL.cols); x++) {
                if ((mRefMap)[y * (mImageL.cols) + x].x == NO_MATCH) {
                    continue;
                }

                float dx = x - (mRefMap)[y * mImageL.cols + x].x;
                float dy = y - (mRefMap)[y * mImageL.cols + x].y;

                //printf("%f\n",sqrt(float(dx*dx+dy*dy)));
                temp[y * mImageL.cols + x] = sqrt((dx * dx + dy * dy));
                cv::Point2f left(x, y);
                cv::Point2f right((mRefMap)[y * mImageL.cols + x].x, (mRefMap)[y * mImageL.cols + x].y);

                features.first.push_back(left);
                features.second.push_back(right);
                if (temp[y * mImageL.cols + x] < mindsp) mindsp = temp[y * mImageL.cols + x];
                if (temp[y * mImageL.cols + x] > maxdsp) maxdsp = temp[y * mImageL.cols + x];
                counter++;
            }
        }
        std::cout << "Depth Inlier: " << counter << std::endl;
        cv::Mat imgtemp(temp);
        imgtemp.reshape(1, mImageL.rows);
        output = imgtemp;
        return features;
    }

    pair<vector<cv::Point>, vector<cv::Point>>
    QuasiDenseStereo::getDisparityImage(cv::Mat &temp, cv::Mat *img, int numDisparityLevels, bool overlay) {
        int s = (int) img->step;
        unsigned char *imgData = (unsigned char *) img->data;
        float mindsp = 1e9;
        float maxdsp = -1e9;

        float *dsp = new float[img->cols * img->rows];
        memset(dsp, 0, img->cols * img->rows * sizeof(float));

        for (int y = 0; y < img->rows; y++) {
            for (int x = 0; x < img->cols; x++) {
                if ((mRefMap)[y * img->cols + x].x == NO_MATCH) {
                    dsp[y * img->cols + x] = NO_MATCH;
                    continue;
                }

                float dx = x - (mRefMap)[y * img->cols + x].x;
                float dy = y - (mRefMap)[y * img->cols + x].y;

                //printf("%f\n",sqrt(float(dx*dx+dy*dy)));
                dsp[y * img->cols + x] = sqrt(float(dx * dx + dy * dy));

                if (dsp[y * img->cols + x] < mindsp) mindsp = dsp[y * img->cols + x];
                if (dsp[y * img->cols + x] > maxdsp) maxdsp = dsp[y * img->cols + x];
            }
        }

        // change the disparity level scale
        if (numDisparityLevels != -1) {
            mindsp = 0;
            maxdsp = numDisparityLevels;
        }

        for (int y = 0; y < (img->rows); y++) {
            for (int x = 0; x < (img->cols); x++) {

                if (dsp[(y) * img->cols + x] == NO_MATCH) {

                    if (!overlay) {
                        unsigned char DisparityImageVal = (unsigned char) 0;
                        imgData[y * s + x * 6] = DisparityImageVal;
//                    imgData[y*s+x*3+1] = 255;
//                    imgData[y*s+x*3+2] = 255;
                    }
                    continue;
                }

                float ScaledDisparity = 255 - 255.f * ((dsp[(y) * img->cols + x] - mindsp) / (maxdsp - mindsp));
                unsigned char DisparityImageVal = (unsigned char) ScaledDisparity;
                imgData[(y) * s + x * 6 + 1] = DisparityImageVal;
                imgData[(y) * s + x * 6 + 3] = DisparityImageVal;
                imgData[(y) * s + x * 6 + 5] = DisparityImageVal;
            }
        }

        delete[] dsp;
        size_t length = mPoints2DL.size();
        pair<vector<cv::Point>, vector<cv::Point>> features;
        for (int i = 1; i < length; i++) {
            cv::Point left = mPoints2DL[i];
            cv::Point right = mRefMap[left.y * (img->cols) + left.x];
            if (left.x == NO_MATCH) continue;
            if (right.x == NO_MATCH) continue;
            features.first.push_back(left);
            features.second.push_back(right);
        }

        return features;
    }
}