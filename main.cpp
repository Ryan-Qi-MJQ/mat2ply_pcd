#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/grid_minimum.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>

#include <pcl/visualization/pcl_visualizer.h>

using namespace cv;
using namespace std;




class filters {
public:
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    filters() :
            pass_deep_       (1000.0f),
            outlier_meak_    (20),
            outlier_stdm_    (2.8f),
            leaf_size_       (2.0f)
    {}
    ~filters()  {}

    // PassThrough
    void  pass_through_filter ( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud  ) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tem_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud,*tem_cloud);
        pcl::PassThrough<pcl::PointXYZ> cloud_filter; // Create the filtering object
        cloud_filter.setInputCloud(tem_cloud);           // Input generated cloud to filter
        cloud_filter.setFilterFieldName("z");        // Set field name to Z-coordinate
        cloud_filter.setFilterLimits(1.0, pass_deep_);      // Set accepted interval values
        cloud_filter.filter(*cloud);
        std::cout << "After PassThrough filter, point cloud have " << cloud->points.size()
                  << " datas." << std::endl;
        std::cout << "After PassThrough filter, tem_cloud cloud have " << tem_cloud->points.size()
                  << " datas." << std::endl;
    }
    // Statistical Outlier
    void stat_outlier_filter( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud  ) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tem_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud,*tem_cloud);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(tem_cloud);  //    oricloud   cloud_cropbox
        sor.setMeanK( outlier_meak_ );  // 数值小会将过多的点认为是离群点。
        sor.setStddevMulThresh(outlier_stdm_);  // 数值小会将过多的点认为是离群点。
        sor.filter(*cloud);
        std::cout << "After Statistical Outlier Removal filter, point cloud have " << cloud->points.size()
                  << " datas." << std::endl;
    }

    // VoxelGrid 下采样
    void voxel_grid_filter( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tem_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud,*tem_cloud);
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(tem_cloud);
        vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_); // 单位为米(m), 体素叶子大小为0.01
        vg.filter(*cloud);
        std::cout << "PointCloud after VoxelGrid filtering has: " << cloud->points.size() << " data points."
                  << std::endl;
    }

private:
    float pass_deep_ ;
    int outlier_meak_ ;
    float outlier_stdm_ ;
    float leaf_size_ ;
};

int main()
{
    std::string str_loadpath = "/home/cobot/桌面/12345/ImageFromCamera/point_cloud_extrinsic_pose_04_16_16_57_41_525.yaml";
    std::string str_savepath = "/home/cobot/桌面/point_cloud_extrinsic_pose_04_16_16_57_41_525.ply";

    FileStorage fs(str_loadpath,FileStorage::READ);
    if (!fs.isOpened())
    {
        cout<<"No file!"<<endl;
        return false;
    }

    Mat Matrix1;
    fs["data"] >> Matrix1;
    cout << "Matrix1: " << Matrix1.size() << endl;


    //pcl::PointCloud<pcl::PointXYZ>::Ptr test::mat2cloud(cv::Mat &Matrix1)

        pcl::PointCloud<pcl::PointXYZ>::Ptr incloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(int i=0;i<Matrix1.rows;i++){
            for(int j=0;j <Matrix1.cols;j++){
                pcl::PointXYZ pttmp;
                pttmp.x = Matrix1.at<cv::Vec3f>(i,j)[0];
                pttmp.y = Matrix1.at<cv::Vec3f>(i,j)[1];
                pttmp.z = Matrix1.at<cv::Vec3f>(i,j)[2];
                if ( pttmp.x != 0 &&  pttmp.y != 0 &&  pttmp.z != 0 ) {
                incloud->points.push_back(pttmp);
                }
            }
        }
        incloud->width = incloud->points.size();
        incloud->height =1;
        incloud->is_dense = true;

        pcl::visualization::PCLVisualizer viewer ;
//
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (incloud, 0, 50, 50 ) ;
//        viewer.addPointCloud( incloud , color, "cloud") ;


        filters ft ;
        ft.pass_through_filter(incloud);
        ft.stat_outlier_filter(incloud);
        ft.voxel_grid_filter(incloud);

//        pcl::io::savePCDFile(str_savepath,*incloud);
        pcl::io::savePLYFile(str_savepath,*incloud);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1 ( incloud, 255, 255, 255 ) ;
        viewer.addPointCloud( incloud , color1, "cloud1") ;

        while  (! viewer.wasStopped() ) {
            viewer.spin() ;
        }




//    cv::imshow("aaa",Matrix1);
//    cv::waitKey(0);
}