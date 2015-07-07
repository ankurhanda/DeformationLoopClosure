#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <Eigen/CholmodSupport>
#include <nanoflann.hpp>
#include <sstream>
#include <vector_types.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <cvd/image.h>
#include <cvd/image_io.h>

using namespace Eigen;
using namespace std;
using namespace nanoflann;


int main(void)
{
    ifstream ifile("../data/SE3poses.txt");

    std::vector<TooN::SE3<> > gtPoses;

    TooN::SE3<> T_wc;

    while(1)
    {
        ifile >> T_wc;

        if( ifile.eof() )
            break;

        std::cout << T_wc << std::endl;

        gtPoses.push_back(T_wc);
    }

    ifile.close();

    char fileName[300];

    int width = 640;
    int height = 480;

    CVD::Image<u_int16_t>_readImage(CVD::ImageRef(width,height));

//    float depthImage[width*height];

    ofstream ofile("points.obj");

    float fx= 480;
    float fy = -480;

    float u0 = 319.5;
    float v0 = 239.5;

    int count = 0;

    float drift_scale = 5E-3;

    for(int i = 0; i < gtPoses.size(); i+=20)
    {
        sprintf(fileName,"../data/depth/depth_%04d.png",i);

        std::cout<<fileName<<std::endl;

        CVD::img_load(_readImage,fileName);

        count = i;

        TooN::SE3<>T_w_cur = gtPoses.at(i);

        T_w_cur = TooN::SE3<>(T_w_cur.get_rotation(),
                              T_w_cur.get_translation()
                              + TooN::makeVector(drift_scale,drift_scale*0,drift_scale)*count);

        for(int yy = 0; yy < height; yy+=4)
        {
            for(int xx = 0; xx < width; xx+=4)
            {
                if( _readImage[CVD::ImageRef(xx,yy)] == 65535 )
                    continue;

                float depth = (float)(_readImage[CVD::ImageRef(xx,yy)])/5000.0f;

                TooN::Vector<3>point_c = TooN::makeVector(depth*(xx-u0)/fx,
                                                          depth*(yy-v0)/fy,
                                                          depth);

                point_c  = T_w_cur * point_c;

                ofile<<"v "<<point_c[0]<<" "<<point_c[1]<<" "<<point_c[2]<<std::endl;
            }
        }

    }

    ofile.close();



}
