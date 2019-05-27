#ifndef MERGE_TOOL_H_
#define MERGE_TOOL_H_
#ifndef MAX_QUEUE_SIZE
#define MAX_QUEUE_SIZE 5
#endif
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl/point_representation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <velodyne/pcap_cache.hpp>

namespace velodyne {

    class MergeItem {
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
        public:
            int64_t currentCloudIdx;
            PcapCache cache;
            Eigen::Matrix4f transformMatrix;
            std::stack<Eigen::Matrix4f> transformMatrixPre;
            PointCloudPtrT showCloud;
            std::string configFilename;

            MergeItem(std::string outputPath): cache(outputPath), transformMatrix(Eigen::Matrix4f::Identity()) { };

            void saveConfig() {
                std::ofstream ofs(configFilename);

                ofs << transformMatrix << std::endl;
                ofs << currentCloudIdx;
            }

            void loadConfig() {
                std::ifstream ifs(configFilename);

                if(ifs.is_open()){
                    std::string s;
                    for(int i = 0; i < 4; i++) {
                        for(int j = 0; j < 4; j++) {
                            ifs >> s;
                            transformMatrix(i,j) = std::stof(s);
                        }
                    }
                    ifs >> s;
                    currentCloudIdx = std::stoll(s);
                }
            }
    };

    class MergeTool {
        typedef pcl::PointXYZ PointT;
        typedef pcl::PointCloud<PointT> PointCloudT;
        typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
        typedef pcl::PointNormal PointNormalT;
        typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

        public:

            int currentItemIdx;
            std::vector<MergeItem> items;
            boost::shared_ptr<std::mutex> mutex;

            MergeTool(std::mutex &mutex): mutex(&mutex) { };

            void add(std::string filename, PointCloudPtrT &cloud) {
                boost::filesystem::path p{filename};
                boost::filesystem::path m{filename};
                m.replace_extension("txt");
                MergeItem mergeItem("/tmp/" + p.stem().string());
                mergeItem.configFilename = m.string();
                mergeItem.loadConfig();
                mergeItem.cache.addPcap(filename);
                mergeItem.cache.setRange(0,100);
                mergeItem.cache.convert();
                cloud = mergeItem.cache.getCloudById(mergeItem.currentCloudIdx);
                pcl::transformPointCloud(*cloud, *cloud, mergeItem.transformMatrix);
                mergeItem.showCloud = cloud;
                
                std::cout << "Merge tool add: " << p.stem().string()  << "\tid: " << items.size() << std::endl;
                currentItemIdx = (items.size() == 0 ? 0 : 1);
                items.push_back(mergeItem);
            }

            void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
            {
                Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();

                if((event.getKeySym() == "KP_6")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix(0,3) = 10.0;
                    } else if(event.isCtrlPressed()) {
                        matrix(0,3) = 5.0;
                    } else {
                        matrix(0,3) = 1.0;
                    }
                } else if((event.getKeySym() == "KP_4")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix(0,3) = -10.0;
                    } else if(event.isCtrlPressed()) {
                        matrix(0,3) = -5.0;
                    } else {
                        matrix(0,3) = -1.0;
                    }
                } else if((event.getKeySym() == "KP_8")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix(1,3) = 10.0;
                    } else if(event.isCtrlPressed()) {
                        matrix(1,3) = 5.0;
                    } else {
                        matrix(1,3) = 1.0;
                    }
                } else if((event.getKeySym() == "KP_2")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix(1,3) = -10.0;
                    } else if(event.isCtrlPressed()) {
                        matrix(1,3) = -5.0;
                    } else {
                        matrix(1,3) = -1.0;
                    }
                } else if((event.getKeySym() == "KP_5")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix(2,3) = -10.0;
                    } else if(event.isCtrlPressed()) {
                        matrix(2,3) = -5.0;
                    } else {
                        matrix(2,3) = -1.0;
                    }
                } else if((event.getKeySym() == "KP_0")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix(2,3) = 10.0;
                    } else if(event.isCtrlPressed()) {
                        matrix(2,3) = 5.0;
                    } else {
                        matrix(2,3) = 1.0;
                    }
                } else if((event.getKeySym() == "KP_1")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (5 * M_PI / 180.0, Eigen::Vector3f::UnitX()).matrix();
                    } else if(event.isCtrlPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (1 * M_PI / 180.0, Eigen::Vector3f::UnitX()).matrix();
                    } else {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (0.5 * M_PI / 180.0, Eigen::Vector3f::UnitX()).matrix();
                    }
                } else if((event.getKeySym() == "KP_3")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-5 * M_PI / 180.0, Eigen::Vector3f::UnitX()).matrix();
                    } else if(event.isCtrlPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-1 * M_PI / 180.0, Eigen::Vector3f::UnitX()).matrix();
                    } else {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-0.5 * M_PI / 180.0, Eigen::Vector3f::UnitX()).matrix();
                    }
                } else if((event.getKeySym() == "KP_7")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (5 * M_PI / 180.0, Eigen::Vector3f::UnitY()).matrix();
                    } else if(event.isCtrlPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (1 * M_PI / 180.0, Eigen::Vector3f::UnitY()).matrix();
                    } else {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (0.5 * M_PI / 180.0, Eigen::Vector3f::UnitY()).matrix();
                    }
                } else if((event.getKeySym() == "KP_9")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-5 * M_PI / 180.0, Eigen::Vector3f::UnitY()).matrix();
                    } else if(event.isCtrlPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-1 * M_PI / 180.0, Eigen::Vector3f::UnitY()).matrix();
                    } else {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-0.5 * M_PI / 180.0, Eigen::Vector3f::UnitY()).matrix();
                    }
                } else if((event.getKeySym() == "bracketright")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (5 * M_PI / 180.0, Eigen::Vector3f::UnitZ()).matrix();
                    } else if(event.isCtrlPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (1 * M_PI / 180.0, Eigen::Vector3f::UnitZ()).matrix();
                    } else {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (0.5 * M_PI / 180.0, Eigen::Vector3f::UnitZ()).matrix();
                    }
                } else if((event.getKeySym() == "bracketleft")&&(event.keyDown())) {
                    if(event.isShiftPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-5 * M_PI / 180.0, Eigen::Vector3f::UnitZ()).matrix();
                    } else if(event.isCtrlPressed()) {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-1 * M_PI / 180.0, Eigen::Vector3f::UnitZ()).matrix();
                    } else {
                        matrix.block<3,3>(0,0) = Eigen::AngleAxisf (-0.5 * M_PI / 180.0, Eigen::Vector3f::UnitZ()).matrix();
                    }
                } else if((event.getKeySym() == "Up")&&(event.keyDown())) {
                    currentItemIdx = ((currentItemIdx + 1) < items.size() ? (currentItemIdx + 1) : 0);
                    std::cout << "Current file index: " << currentItemIdx << std::endl;
                } else if((event.getKeySym() == "Down")&&(event.keyDown())) {
                    currentItemIdx = ((currentItemIdx - 1) >= 0 ? (currentItemIdx - 1) : items.size() - 1);
                    std::cout << "Current file index: " << currentItemIdx << std::endl;
                } else if((event.getKeySym() == "period")&&(event.keyDown())) {
                    items[currentItemIdx].currentCloudIdx = ((items[currentItemIdx].currentCloudIdx+1) < items[currentItemIdx].cache.end() ? items[currentItemIdx].currentCloudIdx+1 : 0);
                    pcl::transformPointCloud(*items[currentItemIdx].cache.getCloudById(items[currentItemIdx].currentCloudIdx), *items[currentItemIdx].showCloud, items[currentItemIdx].transformMatrix);
                    std::cout << "File " << currentItemIdx << " cloud index: " << items[currentItemIdx].currentCloudIdx << std::endl;
                } else if((event.getKeySym() == "comma")&&(event.keyDown())) {
                    items[currentItemIdx].currentCloudIdx = ((items[currentItemIdx].currentCloudIdx-1) >= 0 ? items[currentItemIdx].currentCloudIdx-1 : items[currentItemIdx].cache.end());
                    pcl::transformPointCloud(*items[currentItemIdx].cache.getCloudById(items[currentItemIdx].currentCloudIdx), *items[currentItemIdx].showCloud, items[currentItemIdx].transformMatrix);
                    std::cout << "File " << currentItemIdx << " cloud index: " << items[currentItemIdx].currentCloudIdx << std::endl;
                } else if((event.getKeySym() == "z")&&(event.keyDown())) {
                    if(items[currentItemIdx].transformMatrixPre.size() > 0) {
                        pcl::transformPointCloud(*items[currentItemIdx].showCloud, *items[currentItemIdx].showCloud, items[currentItemIdx].transformMatrix.inverse().eval());
                        pcl::transformPointCloud(*items[currentItemIdx].showCloud, *items[currentItemIdx].showCloud, items[currentItemIdx].transformMatrixPre.top());
                        items[currentItemIdx].transformMatrix = items[currentItemIdx].transformMatrixPre.top();
                        items[currentItemIdx].transformMatrixPre.pop();
                        std::cout << "Load previous transform matrix (" << items[currentItemIdx].transformMatrixPre.size() << ")" << std::endl;
                    }
                    return;
                } else if((event.getKeySym() == "i")&&(event.keyDown())) {
                    std::cout << "input file index as source(current: " << currentItemIdx << "):";
                    int sourceIdx;
                    cin >> sourceIdx;
                    matrix = easyICP(items[sourceIdx].showCloud, items[currentItemIdx].showCloud, 2, 50);
                } else if(((event.getKeySym() == "h")||(event.getKeySym() == "H"))&&(event.keyDown())) {
                    std::cout << std::endl;
                    std::cout << "\033[1;33m";  //yellow
                    std::cout << "| merge tool added:" << std::endl;
                    std::cout << "\033[0m";     //default color
                    std::cout << "-------         " << std::endl;
                    std::cout << "              6 " << " : translation +x" << std::endl;
                    std::cout << "              4 " << " : translation -x" << std::endl;
                    std::cout << "              8 " << " : translation +y" << std::endl;
                    std::cout << "              2 " << " : translation -y" << std::endl;
                    std::cout << "              0 " << " : translation +z" << std::endl;
                    std::cout << "              5 " << " : translation -z" << std::endl;
                    std::cout << "              1 " << " : rotate +x" << std::endl;
                    std::cout << "              3 " << " : rotate -x" << std::endl;
                    std::cout << "              7 " << " : rotate +y" << std::endl;
                    std::cout << "              9 " << " : rotate -y" << std::endl;
                    std::cout << "              [ " << " : rotate +z" << std::endl;
                    std::cout << "              ] " << " : rotate -z" << std::endl;
                } else {
                    if(event.keyDown())
                        std::cout << "Keyboard pressed: " << ((event.isAltPressed())? "Alt + " : "") << ((event.isCtrlPressed())? "Ctrl + " : "") << ((event.isShiftPressed())? "Shift + " : "") << event.getKeySym() << std::endl;
                    return;
                }
                pcl::transformPointCloud(*items[currentItemIdx].showCloud, *items[currentItemIdx].showCloud, items[currentItemIdx].transformMatrix.inverse().eval());
                items[currentItemIdx].transformMatrixPre.push(items[currentItemIdx].transformMatrix);
                items[currentItemIdx].transformMatrix *= matrix;
                pcl::transformPointCloud(*items[currentItemIdx].showCloud, *items[currentItemIdx].showCloud, items[currentItemIdx].transformMatrix);
                items[currentItemIdx].saveConfig();
                
            }

            Eigen::Matrix4f easyICP (const PointCloudPtrT cloud_src, const PointCloudPtrT cloud_tgt, const int iterPerTime = 2,const int times = 30, bool downsample = false)
            {
                class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
                {
                    using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
                public:
                    MyPointRepresentation ()
                    {
                        // Define the number of dimensions
                        nr_dimensions_ = 4;
                    }

                    // Override the copyToFloatArray method to define our feature vector
                    virtual void copyToFloatArray (const PointNormalT &p, float * out) const
                    {
                        // < x, y, z, curvature >
                        out[0] = p.x;
                        out[1] = p.y;
                        out[2] = p.z;
                        out[3] = p.curvature;
                    }
                };
                //
                // Downsample for consistency and speed
                // \note enable this for large datasets
                PointCloudPtrT src (new PointCloudT);
                PointCloudPtrT tgt (new PointCloudT);
                pcl::VoxelGrid<PointT> grid;
                if (downsample)
                {
                    grid.setLeafSize (0.05, 0.05, 0.05);
                    grid.setInputCloud (cloud_src);
                    grid.filter (*src);

                    grid.setInputCloud (cloud_tgt);
                    grid.filter (*tgt);
                }
                else
                {
                    src = cloud_src;
                    tgt = cloud_tgt;
                }


                // Compute surface normals and curvature
                PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
                PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

                pcl::NormalEstimation<PointT, PointNormalT> norm_est;
                pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
                norm_est.setSearchMethod (tree);
                norm_est.setKSearch (30);
                
                norm_est.setInputCloud (src);
                norm_est.compute (*points_with_normals_src);
                pcl::copyPointCloud (*src, *points_with_normals_src);

                norm_est.setInputCloud (tgt);
                norm_est.compute (*points_with_normals_tgt);
                pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

                //
                // Instantiate our custom point representation (defined above) ...
                MyPointRepresentation point_representation;
                // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
                float alpha[4] = {1.0, 1.0, 1.0, 1.0};
                point_representation.setRescaleValues (alpha);

                //
                // Align
                pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
                reg.setTransformationEpsilon (1e-6);
                // Set the maximum distance between two correspondences (src<->tgt) to 10cm
                // Note: adjust this based on the size of your datasets
                reg.setMaxCorrespondenceDistance (100);  
                // Set the point representation
                reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

                reg.setInputSource (points_with_normals_src);
                reg.setInputTarget (points_with_normals_tgt);



                //
                // Run the same optimization in a loop and visualize the results
                Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
                PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
                reg.setMaximumIterations (iterPerTime);
                for (int i = 0; i < times; ++i)
                {
                    // save cloud for visualization purpose
                    points_with_normals_src = reg_result;

                    // Estimate
                    reg.setInputSource (points_with_normals_src);
                    reg.align (*reg_result);

                    //accumulate transformation between each Iteration
                    Ti = reg.getFinalTransformation () * Ti;

                    //if the difference between this transformation and the previous one
                    //is smaller than the threshold, refine the process by reducing
                    //the maximal correspondence distance
                    if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
                    reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.1);
                    
                    prev = reg.getLastIncrementalTransformation ();

                }

                //
                // Get the transformation from target to source
                targetToSource = Ti.inverse();

                return targetToSource;
            }

    };
}

#endif