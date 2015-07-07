#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <Eigen/CholmodSupport>
#include <nanoflann.hpp>
#include <sstream>
#include <vector_types.h>
#include <vector_functions.h>


/// PCL GARBAGE
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <boost/chrono.hpp>
#include <boost/timer.hpp>
#include <TooN/TooN.h>
#include <Eigen/Eigenvalues>

#include <cvd/image.h>
#include <cvd/image_io.h>


#include <gvars3/instances.h>
#include <TooN/se3.h>

using namespace GVars3;
using namespace Eigen;
using namespace std;
using namespace nanoflann;
using namespace CVD;

/// Good reference
/// http://scicomp.stackexchange.com/questions/8147/library-that-performs-sparse-matrix-vector-and-matrix-transpose-vector-multiplic

/// Tutorial for Eigen Cholmod module
/// http://eigen.tuxfamily.org/dox/group__TopicSparseSystems.html

// This is an exampleof a custom data set class

// This is an exampleof a custom data set class
template <typename T>
struct PointCloud
{
    struct Point
    {
        T  x,y,z;
    };

    std::vector<Point>  pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
    {
        const T d0=p1[0]-pts[idx_p2].x;
        const T d1=p1[1]-pts[idx_p2].y;
        const T d2=p1[2]-pts[idx_p2].z;
        return d0*d0+d1*d1+d2*d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim==0) return pts[idx].x;
        else if (dim==1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

};


void change_basis(TooN::SE3<>& T_wc_ref,
                  TooN::Matrix<4>&T)
{
    TooN::Matrix<4>T4x4 = T.T() * T_wc_ref * T  ;

    TooN::Matrix<3>R_slice = TooN::Data(T4x4(0,0),T4x4(0,1),T4x4(0,2),
                                        T4x4(1,0),T4x4(1,1),T4x4(1,2),
                                        T4x4(2,0),T4x4(2,1),T4x4(2,2));


    TooN::Vector<3>t_slice = TooN::makeVector(T4x4(0,3),T4x4(1,3),T4x4(2,3));

    T_wc_ref = TooN::SE3<>(TooN::SO3<>(R_slice),t_slice);

}

int main(void)
{

    GUI.LoadFile("../data/variables.cfg");

    std::ifstream ifile("../data/points.obj");

    char readlinedata[300];
    char c;
    float x_coord, y_coord, z_coord;

    PointCloud<float>points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr model (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());

    while(1)
    {
        ifile.get(c);
        ifile.getline(readlinedata,300);

        if ( ifile.eof() )
            break;

        istringstream iss(readlinedata);

        iss >> x_coord; iss >> y_coord; iss >> z_coord;

        pcl::PointXYZ pt;

        pt.x = x_coord;
        pt.y = y_coord;
        pt.z = z_coord;

        model->push_back(pt);
    }

    ifile.close();

    std::cout<<"Total number of points = " << model->size() << std::endl;

    boost::timer timer;

    pcl::PointCloud<int> sampled_indices;
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;

    float model_ss_(0.5f);

    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    uniform_sampling.compute (sampled_indices);
    pcl::copyPointCloud (*model, sampled_indices.points, *model_keypoints);

    std::cout << "Model total points: " << model->size ()
              << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

    ofstream ofile("sparse_skeleton.obj");

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = model_keypoints->begin();
                                            it != model_keypoints->end(); it++)

    {
        ofile << "v "<< it->x << " " << it->y << " " << it->z << endl;

        PointCloud<float>::Point pt;

        pt.x = it->x;
        pt.y = it->y;
        pt.z = it->z;

        points.pts.push_back(pt);
    }

    std::cout<<"elapsed  = " << timer.elapsed() << std::endl;


    ifstream posefile("../data/SE3poses.txt");

    std::vector<TooN::SE3<> > gtPoses;

    TooN::SE3<> T_wc;

    while(1)
    {
        posefile >> T_wc;

        if( posefile.eof() )
            break;

        gtPoses.push_back(T_wc);
    }


    std::cout<<"number of poses " << gtPoses.size() << std::endl;

    ifile.close();

    int ref_frame  = GV3::get<int>("start");
    int live_frame = GV3::get<int>("end");

    std::string baseDir  = GV3::get<std::string>("basedir");


    std::cout << std::endl;
    std::cout << "ref_frame = " << ref_frame  << std::endl;
    std::cout << "live_frame= " << live_frame << std::endl;
    std::cout << "basedir = " << baseDir << std::endl;


    /// Read the reference image called destination image

    char DepthFileName[300];

    int width  = 640;
    int height = 480;

    TooN::SE3<>T_w_ref   = gtPoses.at(ref_frame);
    TooN::SE3<>T_w_live  = gtPoses.at(live_frame);

    TooN::SE3<>T_w_ref_noisy = TooN::SE3<>(T_w_ref.get_rotation(),
                                           T_w_ref.get_translation()
                                          + TooN::makeVector(5E-3,0,5E-3)*ref_frame);

    TooN::SE3<>T_w_live_noisy = TooN::SE3<>(T_w_live.get_rotation(),
                                            T_w_live.get_translation()
                                          + TooN::makeVector(5E-3,0,5E-3)*live_frame);


    TooN::SE3<>T_ref_live = T_w_ref.inverse() * T_w_live;

    sprintf(DepthFileName,"%s/depth_%04d.png",baseDir.c_str(),ref_frame);

    std::cout<<"fileName = " << DepthFileName << std::endl;

    CVD::Image<u_int16_t>_referenceDepthMap(CVD::ImageRef(width,height));
    CVD::img_load(_referenceDepthMap,DepthFileName);


    sprintf(DepthFileName,"%s/depth_%04d.png",baseDir.c_str(),live_frame);
    CVD::Image<u_int16_t>_liveDepthMap(CVD::ImageRef(width,height));
    CVD::img_load(_liveDepthMap,DepthFileName);

    std::cout<<"fileName = " << DepthFileName << std::endl;

    float fx  =  480.0;
    float fy  = -480.0;
    float u0  =  319.5;
    float v0  =  239.5;

    std::vector<bool>correspondence(width*height,false);

    int total_constraints=0;

    for(int yy = 0; yy < height; yy++)
    {
        for(int xx = 0; xx < width; xx++)
        {
            float depth = (float)_liveDepthMap[CVD::ImageRef(xx,yy)]/5000.0f;

            TooN::Vector<3>point_live = TooN::makeVector(depth*(xx-u0)/fx,
                                                         depth*(yy-v0)/fy,
                                                         depth);

            TooN::Vector<3>p_ref_live = T_ref_live * point_live;

            int2 _pref = make_int2(fx * (p_ref_live[0]/p_ref_live[2]) + u0 /*+ 0.5*/,
                                   fy * (p_ref_live[1]/p_ref_live[2]) + v0 /*+ 0.5*/);

            if ( _pref.x >= 0 && _pref.x < width &&
                 _pref.y >= 0 && _pref.y < height )
            {
                float depth_ref = (float)_referenceDepthMap[CVD::ImageRef(_pref.x,_pref.y)]/5000.0f;

                if ( fabs( depth_ref - p_ref_live[2] ) < 1e-2 )
                {
                    correspondence.at(xx+yy*width) = true;

                    total_constraints++;

                    TooN::Vector<3>Q_s_w = T_w_live_noisy * point_live;

                    ofile <<"v "<<Q_s_w[0]<<", "<<Q_s_w[1]<<", "<<Q_s_w[2]
                          <<" "<<0<<" "<<1<<" "<<0 << std::endl;

                    TooN::Vector<3>point_ref = TooN::makeVector(depth_ref*(_pref.x-u0)/fx ,
                                                                depth_ref*(_pref.y-v0)/fy,
                                                                depth_ref);

                    TooN::Vector<3>Q_d_w = T_w_ref_noisy * point_ref;

                    ofile <<"v "<<Q_d_w[0]<<", "<<Q_d_w[1]<<", "<<Q_d_w[2]
                          <<" "<<1<<" "<<0<<" "<<0 << std::endl;
                }
            }

        }
    }

    std::cout<<"T_w_ref        =  " << T_w_ref << std::endl;
    std::cout<<"T_w_ref_noisy  = " << T_w_ref_noisy << std::endl;
    std::cout<<"T_w_live       = " << T_w_live << std::endl;
    std::cout<<"T_w_live_noisy = " << T_w_live_noisy << std::endl;

    ofile.close();

    std::cout<<"number of correspondences = " << total_constraints << std::endl;

    Eigen::SparseMatrix<double>A_lm(6*points.pts.size(),6*points.pts.size());

    for(int i = 0; i < 6*points.pts.size();i++)
    {
        A_lm.insert(i,i) = 1;
    }

    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<float, PointCloud<float> > ,
        PointCloud<float>,
        3 /* dim */
        > my_kd_tree_t;

    my_kd_tree_t   this_kdtree(3 /*dim*/,
                               points,
                               KDTreeSingleIndexAdaptorParams(10 /* max leaf */) );

    this_kdtree.buildIndex();

    const size_t num_results = 5;
    std::vector<size_t>   ret_index(num_results);
    std::vector<float> out_dist_sqr(num_results);


    Eigen::SparseMatrix<double> A_reg(12*points.pts.size(),
                                       6*points.pts.size());


    Eigen::VectorXd b_reg(12*points.pts.size());


    PointCloud<float>::Point g_k;
    PointCloud<float>::Point g_j;
    float w3, w2, w1;

    std::vector<TooN::SO3<> > R_matrices(points.pts.size());
    std::vector<TooN::Vector<3> > translations(points.pts.size());

    for(int i=0; i < translations.size(); i++)
        translations.at(i) = TooN::makeVector(0.0,0.0,0.0);

    std::cout<<"Inserting the points... " << std::endl;

    int itr = 0;

    while(1)
    {
        if ( itr >= 35  )
            break;

        itr++;

        /// Add the regularisation constraints...
        for(int j = 0; j < points.pts.size(); j++)
        {

            /// Find out the 4 nearest neighbours of this point
            /// and add regularisation to that.

            TooN::Vector<3>t_j      = translations.at(j);

            const float query_pt[3] = {points.pts[j].x,
                                       points.pts[j].y,
                                       points.pts[j].z};

            /// We need to compute these only once and save them...

            this_kdtree.knnSearch(&query_pt[0],
                                   num_results,
                                  &ret_index[0],
                                  &out_dist_sqr[0]);

            /// Constraints for j

            g_j = points.pts[j];

            /// The zeroth neighbour is the point itself
            for(int knbr = 1; knbr < 5; knbr++)
            {
                int k = ret_index[knbr];

                g_k = points.pts[k];

                /// Find out the translation
                TooN::Vector<3>t_k   = translations.at(k);

                w3 = g_k.z - g_j.z;
                w2 = g_k.y - g_j.y;
                w1 = g_k.x - g_j.x;

                /// Multiply this difference with the Rotation matrix
                /// corresponding to j

                TooN::Vector<3>Rgkgj = R_matrices.at(j) * TooN::makeVector(w1,w2,w3);

                w3 = Rgkgj[2];
                w2 = Rgkgj[1];
                w1 = Rgkgj[0];

                /// Insert a 3x3 Rotation matrix here for each ret_index
                /// Insert an Identity matrix I and -I wherever possible

                /// we should also += instead if same vertices are involved in
                /// different constraints.


                int nbr = knbr-1;

                A_reg.insert(12*j+3*nbr+0, 6*j+0) =  0;
                A_reg.insert(12*j+3*nbr+0, 6*j+1) =  w3;
                A_reg.insert(12*j+3*nbr+0, 6*j+2) = -w2;

                A_reg.insert(12*j+3*nbr+1, 6*j+0) = -w3;
                A_reg.insert(12*j+3*nbr+1, 6*j+1) = 0;
                A_reg.insert(12*j+3*nbr+1, 6*j+2) = w1;

                A_reg.insert(12*j+3*nbr+2, 6*j+0) = w2;
                A_reg.insert(12*j+3*nbr+2, 6*j+1) = -w1;
                A_reg.insert(12*j+3*nbr+2, 6*j+2) = 0;

                A_reg.insert(12*j+3*nbr+0, 6*j+3) = 1;
                A_reg.insert(12*j+3*nbr+1, 6*j+4) = 1;
                A_reg.insert(12*j+3*nbr+2, 6*j+5) = 1;

                /// Make sure this is k.

                A_reg.insert(12*j+3*nbr+0, 6*k+3) = -1;
                A_reg.insert(12*j+3*nbr+1, 6*k+4) = -1;
                A_reg.insert(12*j+3*nbr+2, 6*k+5) = -1;

                b_reg(12*j+3*nbr+0) = Rgkgj[0] + t_j[0]-t_k[0] + g_j.x - g_k.x;
                b_reg(12*j+3*nbr+1) = Rgkgj[1] + t_j[1]-t_k[1] + g_j.y - g_k.y;
                b_reg(12*j+3*nbr+2) = Rgkgj[2] + t_j[2]-t_k[2] + g_j.z - g_k.z;

            }
        }

        /// Add the loop closure constraints here...

        Eigen::SparseMatrix<double>A_con(3*total_constraints,6*points.pts.size());
        Eigen::VectorXd b_con(3*total_constraints);

        int constraint=0;

        for(int yy = 0; yy < height; yy += 8 )
        {
            for(int xx = 0; xx < width; xx += 8 )
            {
                int i = constraint;

                if ( !correspondence.at(yy*width+xx) )
                    continue;

                PointCloud<float>::Point Q_d;
                PointCloud<float>::Point Q_s;

                float depth_live = (float)_liveDepthMap[CVD::ImageRef(xx,yy)]/5000.0f;

                Q_s.x = ((xx - u0)/fx)*depth_live;
                Q_s.y = ((yy - v0)/fy)*depth_live;
                Q_s.z = depth_live;

                TooN::Vector<3>p_s2d = T_ref_live * TooN::makeVector(Q_s.x,
                                                                     Q_s.y,
                                                                     Q_s.z);

                int2 _pref = make_int2(fx * (p_s2d[0]/p_s2d[2]) + u0/* + 0.5f*/,
                                       fy * (p_s2d[1]/p_s2d[2]) + v0/* + 0.5f*/);

                float depth_ref = (float)_referenceDepthMap[CVD::ImageRef(_pref.x,_pref.y)]/5000.0f;

                Q_d.x = ( (_pref.x-u0)/fx ) * depth_ref;
                Q_d.y = ( (_pref.y-v0)/fy ) * depth_ref;
                Q_d.z = depth_ref;

                TooN::Vector<3>Q_s_w = T_w_live_noisy * TooN::makeVector(Q_s.x,
                                                                         Q_s.y,
                                                                         Q_s.z);
                Q_s.x = Q_s_w[0];
                Q_s.y = Q_s_w[1];
                Q_s.z = Q_s_w[2];


                TooN::Vector<3>Q_d_w = T_w_ref_noisy * TooN::makeVector(Q_d.x,
                                                                        Q_d.y,
                                                                        Q_d.z);

                Q_d.x = Q_d_w[0];
                Q_d.y = Q_d_w[1];
                Q_d.z = Q_d_w[2];

                const float query_pt[3] = {Q_s.x,
                                           Q_s.y,
                                           Q_s.z};

                const size_t num_results = 5;
                std::vector<size_t>   ret_index(num_results);
                std::vector<float>    out_dist_sqr(num_results);

                this_kdtree.knnSearch(&query_pt[0], num_results,
                                      &ret_index[0], &out_dist_sqr[0]);

                float w_ki = 0;
                float sum_w_ki = 0;

                TooN::Vector<3>sum_Rvks_gj = TooN::makeVector(0.0,0.0,0.0);

                /// Compute the weights and sum weight
                for(int nbr = 0; nbr < 4; nbr++)
                {
                    int k = ret_index[nbr];

                    PointCloud<float>::Point g_k = points.pts[k];

                    float pt_distance =   (Q_s.x - g_k.x)*(Q_s.x - g_k.x)
                                        + (Q_s.y - g_k.y)*(Q_s.y - g_k.y)
                                        + (Q_s.z - g_k.z)*(Q_s.z - g_k.z);

                    float w_ki_ = (1 -  sqrt(pt_distance/out_dist_sqr[4]) );

                    w_ki = w_ki_ * w_ki_;

                    sum_w_ki += w_ki;
                }

                for(int nbr = 0; nbr < 4; nbr++)
                {
                    int k = ret_index[nbr];

                    PointCloud<float>::Point g_k = points.pts[k];

                    float pt_distance =   (Q_s.x - g_k.x)*(Q_s.x - g_k.x)
                                        + (Q_s.y - g_k.y)*(Q_s.y - g_k.y)
                                        + (Q_s.z - g_k.z)*(Q_s.z - g_k.z);

                    float w_ki_ = (1 - sqrt(pt_distance/out_dist_sqr[4]) );


                    w_ki = w_ki_ * w_ki_;

                    TooN::Vector<3>Rvks_gj = R_matrices.at(k)*TooN::makeVector((Q_s.x - g_k.x),
                                                                               (Q_s.y - g_k.y),
                                                                               (Q_s.z - g_k.z));

                    w1 = Rvks_gj[0];
                    w2 = Rvks_gj[1];
                    w3 = Rvks_gj[2];

                    A_con.insert(3*i+0,6*k+0) =  0;
                    A_con.insert(3*i+0,6*k+1) =  w3*w_ki/sum_w_ki;
                    A_con.insert(3*i+0,6*k+2) = -w2*w_ki/sum_w_ki;

                    A_con.insert(3*i+1,6*k+0) = -w3*w_ki/sum_w_ki;
                    A_con.insert(3*i+1,6*k+1) =  0;
                    A_con.insert(3*i+1,6*k+2) =  w1*w_ki/sum_w_ki;

                    A_con.insert(3*i+2,6*k+0) =  w2*w_ki/sum_w_ki;
                    A_con.insert(3*i+2,6*k+1) = -w1*w_ki/sum_w_ki;
                    A_con.insert(3*i+2,6*k+2) =  0;

                    A_con.insert(3*i+0,6*k+3) = w_ki/sum_w_ki;
                    A_con.insert(3*i+1,6*k+4) = w_ki/sum_w_ki;
                    A_con.insert(3*i+2,6*k+5) = w_ki/sum_w_ki;

                    sum_Rvks_gj += w_ki * (Rvks_gj + translations.at(k) + TooN::makeVector(g_k.x,
                                                                                           g_k.y,
                                                                                           g_k.z));

                }

                sum_Rvks_gj = sum_Rvks_gj / sum_w_ki;

                b_con(3*i+0) = sum_Rvks_gj[0] - Q_d.x;
                b_con(3*i+1) = sum_Rvks_gj[1] - Q_d.y;
                b_con(3*i+2) = sum_Rvks_gj[2] - Q_d.z;

                constraint++;

            }
        }


        /// Shall we add the pin constraints?
        Eigen::SparseMatrix<double>A_pin(3*total_constraints,6*points.pts.size());
        Eigen::VectorXd b_pin(3*total_constraints);

        constraint=0;

        for(int yy = 0; yy < height; yy += 8 )
        {
            for(int xx = 0; xx < width; xx += 8 )
            {
                int i = constraint;

                if ( !correspondence.at(yy*width+xx) )
                    continue;

                PointCloud<float>::Point Q_d;
                PointCloud<float>::Point Q_s;

                float depth_live = (float)_liveDepthMap[CVD::ImageRef(xx,yy)]/5000.0f;

                Q_s.x = ((xx - u0)/fx)*depth_live;
                Q_s.y = ((yy - v0)/fy)*depth_live;
                Q_s.z = depth_live;

                TooN::Vector<3>p_s2d = T_ref_live * TooN::makeVector(Q_s.x,
                                                                     Q_s.y,
                                                                     Q_s.z);

                int2 _pref = make_int2(fx * (p_s2d[0]/p_s2d[2]) + u0 + 0.5f,
                                       fy * (p_s2d[1]/p_s2d[2]) + v0 + 0.5f);

                float depth_ref = (float)_referenceDepthMap[CVD::ImageRef(_pref.x,_pref.y)]/5000.0f;

                Q_d.x = ( (_pref.x-u0)/fx ) * depth_ref;
                Q_d.y = ( (_pref.y-v0)/fy ) * depth_ref;
                Q_d.z = depth_ref;

                TooN::Vector<3>Q_s_w = T_w_live_noisy * TooN::makeVector(Q_s.x,
                                                                         Q_s.y,
                                                                         Q_s.z);
                Q_s.x = Q_s_w[0];
                Q_s.y = Q_s_w[1];
                Q_s.z = Q_s_w[2];


                TooN::Vector<3>Q_d_w = T_w_ref_noisy * TooN::makeVector(Q_d.x,
                                                                        Q_d.y,
                                                                        Q_d.z);

                Q_d.x = Q_d_w[0];
                Q_d.y = Q_d_w[1];
                Q_d.z = Q_d_w[2];

                const float query_pt[3] = {Q_d.x,
                                           Q_d.y,
                                           Q_d.z};

                const size_t num_results = 5;
                std::vector<size_t>   ret_index(num_results);
                std::vector<float>    out_dist_sqr(num_results);

                this_kdtree.knnSearch(&query_pt[0], num_results,
                                      &ret_index[0], &out_dist_sqr[0]);

                float w_ki = 0;
                float sum_w_ki = 0;

                TooN::Vector<3>sum_Rvks_gj = TooN::makeVector(0.0,0.0,0.0);

                /// Compute the weights and sum weight
                for(int nbr = 0; nbr < 4; nbr++)
                {
                    int k = ret_index[nbr];

                    PointCloud<float>::Point g_k = points.pts[k];

                    float pt_distance =   (Q_d.x - g_k.x)*(Q_d.x - g_k.x)
                                        + (Q_d.y - g_k.y)*(Q_d.y - g_k.y)
                                        + (Q_d.z - g_k.z)*(Q_d.z - g_k.z);

                    float w_ki_ = (1 -  sqrt(pt_distance/out_dist_sqr[4]) );

                    w_ki = w_ki_ * w_ki_;

                    sum_w_ki += w_ki;
                }

                for(int nbr = 0; nbr < 4; nbr++)
                {
                    int k = ret_index[nbr];

                    PointCloud<float>::Point g_k = points.pts[k];

                    float pt_distance =   (Q_d.x - g_k.x)*(Q_d.x - g_k.x)
                                        + (Q_d.y - g_k.y)*(Q_d.y - g_k.y)
                                        + (Q_d.z - g_k.z)*(Q_d.z - g_k.z);

                    float w_ki_ = (1 - sqrt(pt_distance/out_dist_sqr[4]) );


                    w_ki = w_ki_ * w_ki_;

                    TooN::Vector<3>Rvks_gj = R_matrices.at(k)*TooN::makeVector((Q_d.x - g_k.x),
                                                                               (Q_d.y - g_k.y),
                                                                               (Q_d.z - g_k.z));

                    w1 = Rvks_gj[0];
                    w2 = Rvks_gj[1];
                    w3 = Rvks_gj[2];

                    A_pin.insert(3*i+0,6*k+0) =  0;
                    A_pin.insert(3*i+0,6*k+1) =  w3*w_ki/sum_w_ki;
                    A_pin.insert(3*i+0,6*k+2) = -w2*w_ki/sum_w_ki;

                    A_pin.insert(3*i+1,6*k+0) = -w3*w_ki/sum_w_ki;
                    A_pin.insert(3*i+1,6*k+1) =  0;
                    A_pin.insert(3*i+1,6*k+2) =  w1*w_ki/sum_w_ki;

                    A_pin.insert(3*i+2,6*k+0) =  w2*w_ki/sum_w_ki;
                    A_pin.insert(3*i+2,6*k+1) = -w1*w_ki/sum_w_ki;
                    A_pin.insert(3*i+2,6*k+2) =  0;

                    A_pin.insert(3*i+0,6*k+3) = w_ki/sum_w_ki;
                    A_pin.insert(3*i+1,6*k+4) = w_ki/sum_w_ki;
                    A_pin.insert(3*i+2,6*k+5) = w_ki/sum_w_ki;

                    sum_Rvks_gj += w_ki * (Rvks_gj + translations.at(k) + TooN::makeVector(g_k.x,
                                                                                           g_k.y,
                                                                                           g_k.z));

                }

                sum_Rvks_gj = sum_Rvks_gj / sum_w_ki;

                b_pin(3*i+0) = sum_Rvks_gj[0] - Q_d.x;
                b_pin(3*i+1) = sum_Rvks_gj[1] - Q_d.y;
                b_pin(3*i+2) = sum_Rvks_gj[2] - Q_d.z;

                constraint++;

            }
        }


        std::cout<<"points have been inserted" << std::endl;

        float w_reg = 1;
        float w_con = 100;
        float w_lm  = 1E-12;

        boost::timer solver_timer;

        Eigen::CholmodSimplicialLDLT< Eigen::SparseMatrix<double> >solver;
        solver.compute(w_reg * A_reg.transpose()*A_reg + w_con * A_con.transpose()*A_con + w_lm*A_lm
                       + w_con * A_pin.transpose()*A_pin);

        if(solver.info()!=Success)
        {
            /// decomposition failed
            std::cout<<"Decomposition failed" << std::endl;
            return 1;
        }

        VectorXd x_update;

        VectorXd Axb = -1.0f*(w_reg * A_reg.transpose()*b_reg + w_con * A_con.transpose()*b_con
                              + w_con*A_pin.transpose()*b_pin);

        x_update = solver.solve(Axb);

        std::cout<<"norm of x_update = " << x_update.norm() << std::endl;

        std::cout<<"solver time elapsed = " << solver_timer.elapsed() << std::endl;

        /// Updates
        for(int i = 0 ; i < points.pts.size();i++)
        {
            /// Get the rotation update
            TooN::Vector<3> so3_i = TooN::makeVector(x_update(6*i+0),
                                                     x_update(6*i+1),
                                                     x_update(6*i+2));

            /// Get the translation update
            TooN::Vector<3> t_i   = TooN::makeVector(x_update(6*i+3),
                                                     x_update(6*i+4),
                                                     x_update(6*i+5));

              R_matrices.at(i)    = TooN::SO3<>(so3_i)*R_matrices.at(i);

            translations.at(i)    = translations.at(i) + t_i;

            TooN::SE3<>pose       = TooN::SE3<>(R_matrices.at(i),translations.at(i));

//            std::cout<<"pose("<<i<<") = "<< pose << std::endl;
        }

    }

    ofstream lc_file("loop_closed.obj");

    for(int i = 0; i < model->size(); i++)
    {
        pcl::PointXYZ p_i = model->at(i);

        TooN::Vector<3>_p_i = TooN::makeVector(p_i.x,p_i.y,p_i.z);

        const size_t num_results = 5;
        std::vector<size_t>   ret_index(num_results);
        std::vector<float>    out_dist_sqr(num_results);

        float query_pt[3] = {_p_i[0],
                             _p_i[1],
                             _p_i[2]};

        this_kdtree.knnSearch(&query_pt[0],     num_results,
                              &ret_index[0], &out_dist_sqr[0]);


        float w_ki = 0;
        float sum_w_ki=0;

        TooN::Vector<3>sum_Rvks_gj = TooN::makeVector(0.0,0.0,0.0);

        for(int nbr = 0; nbr < 4 ; nbr++)
        {
            int k = ret_index[nbr];

             PointCloud<float>::Point g_k = points.pts[k];

             float pt_distance =   (_p_i[0] - g_k.x)*(_p_i[0] - g_k.x)
                                 + (_p_i[1] - g_k.y)*(_p_i[1] - g_k.y)
                                 + (_p_i[2] - g_k.z)*(_p_i[2] - g_k.z);

             float w_ki_ = (1 -  sqrt(pt_distance/out_dist_sqr[4]) );

             w_ki = w_ki_ * w_ki_;

             TooN::Vector<3>Rvks_gj = R_matrices.at(k)*TooN::makeVector((_p_i[0] - g_k.x),
                                                                        (_p_i[1] - g_k.y),
                                                                        (_p_i[2] - g_k.z));


             sum_Rvks_gj += w_ki*(Rvks_gj + TooN::makeVector(g_k.x,
                                                             g_k.y,
                                                             g_k.z) + translations.at(k));

             sum_w_ki    += w_ki;

        }

        sum_Rvks_gj = sum_Rvks_gj / sum_w_ki;

        lc_file <<"v "<<sum_Rvks_gj[0]<<", "<<sum_Rvks_gj[1]<<", "<<sum_Rvks_gj[2]<<std::endl;
    }

    lc_file.close();

    std::cout<<"Success" << std::endl;

}
