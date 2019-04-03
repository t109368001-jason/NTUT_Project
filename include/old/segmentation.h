#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

#include <iostream>
#include <future>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/octree/octree_pointcloud_changedetector.h>

namespace myClass
{
	template<typename PointT = pcl::PointXYZ>
	class objectSegmentation
	{
			string camera_O;
			
			double camera_Width;
			double camera_Height;
			double camera_Depth;
			double camera_offset;

			double segmentation_Up_Bound;
			double segmentation_Down_Bound;
			double segmentation_Left_Bound;
			double segmentation_Right_Bound;
			double segmentation_Scale_Fix;

			bool cameraParameterIsSet;
			bool BoundIsSet;

			bool keep_Inside;

		public:
			objectSegmentation()
			{
				this->BoundIsSet = false;
				this->cameraParameterIsSet = false;
			}

			objectSegmentation(const string &camera_O, const double &camera_Width, const double &camera_Height, const double &camera_FOV, const double &depth_Camera_FOV, const double camera_offset = 0)
			{
				setCameraParameter(camera_O, camera_Width, camera_Height, camera_FOV, depth_Camera_FOV);
			}

			bool setCameraParameter(const string &camera_O, const double &camera_Width, const double &camera_Height, const double &camera_FOV, const double &depth_Camera_FOV, const double camera_offset = 0)
			{
				this->camera_O = camera_O;
				this->camera_Width = camera_Width;
				this->camera_Height = camera_Height;
				this->camera_Depth = (camera_Width/2.0) / tan(camera_FOV/2.0);
				this->camera_offset = camera_offset;

				if(depth_Camera_FOV != 0)		//convert 3D FOV to 2D FOV
				{
					this->segmentation_Scale_Fix = (2 * this->camera_Depth * tan(depth_Camera_FOV / 2.0)) / camera_Width;
				}
				else
				{
					this->segmentation_Scale_Fix = 1.0;
				}

				this->cameraParameterIsSet = true;
				this->BoundIsSet = false;				//須重新計算邊界
				return true;
			}

			//x and y is center
			bool setBound(const double &object_X, const double &object_Y, const double &object_Width, const double &object_Height)
			{
				if(!this->cameraParameterIsSet)
				{
					std::cerr << "Camera parameter is not set" << std::endl;
					return false;
				}
				double object_X_Temp = object_X;
				double object_Y_Temp = object_Y;

				if(this->camera_O == "UL")
				{
					object_X_Temp -= this->camera_Width / 2.0;
					object_Y_Temp -= this->camera_Height / 2.0;
				}

				//convert yolo XYWH to 2D XYWH
				this->segmentation_Left_Bound = (object_X_Temp - (object_Width/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Right_Bound = (object_X_Temp + (object_Width/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Up_Bound = (object_Y_Temp - (object_Height/2.0)) * this->segmentation_Scale_Fix;
				this->segmentation_Down_Bound = (object_Y_Temp + (object_Height/2.0)) * this->segmentation_Scale_Fix;
				
				this->BoundIsSet = true;
				return true;
			}

			void printBound()
			{
				std::cerr << "segmentation_Left_Bound: " << this->segmentation_Left_Bound << std::endl;
				std::cerr << "segmentation_Right_Bound: " << this->segmentation_Right_Bound << std::endl;
				std::cerr << "segmentation_Up_Bound: " << this->segmentation_Up_Bound << std::endl;
				std::cerr << "segmentation_Down_Bound: " << this->segmentation_Down_Bound << std::endl;
			}

			typename pcl::PointCloud<PointT>::Ptr division(const typename pcl::PointCloud<PointT>::Ptr &input, const bool keep_Inside = true)
			{
				if(!BoundIsSet)
				{
					std::cerr << "\nBound parameter is not set\n";
					return input;
				}

				typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
				this->keep_Inside = keep_Inside;
				
                size_t divisionNumber = std::ceil(input->points.size() / (std::thread::hardware_concurrency()+1));

				cloud->points = divisionPart(divisionNumber, input->points.begin(), input->points.end());

				cloud->width = (int) cloud->points.size();
				cloud->height = 1;
				return cloud;
			}

			typename pcl::PointCloud<PointT>::Ptr division_one_thread(const typename pcl::PointCloud<PointT>::Ptr &input, const bool keep_Inside = true)
			{
				if(!BoundIsSet)
				{
					std::cerr << "\nBound parameter is not set\n";
					return input;
				}

				typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
				this->keep_Inside = keep_Inside;
			
				if(this->keep_Inside)
				{
					for(auto it = input->points.begin(); it != input->points.end(); ++it)
					{
						if(this->pointIsInside((*it)))
						{
							cloud->points.push_back(*it);
						}
					}
				}
				else
				{
					for(auto it = input->points.begin(); it != input->points.end(); ++it)
					{
						if(!this->pointIsInside((*it)))
						{
							cloud->points.push_back(*it);
						}
					}
				}
				cloud->width = (int) cloud->points.size();
				cloud->height = 1;
				return cloud;
			}

			typename pcl::PointCloud<PointT>::Ptr division(const pcl::visualization::Camera &camera, const typename pcl::PointCloud<PointT>::Ptr &input, const bool keep_Inside = true)
			{
				if(!BoundIsSet)
				{
					std::cerr << "\nBound parameter is not set\n";
					return input;
				}

				typename pcl::PointCloud<PointT>::Ptr cloud(new typename pcl::PointCloud<PointT>);
				this->keep_Inside = keep_Inside;

                size_t divisionNumber = std::ceil(input->points.size() / (std::thread::hardware_concurrency()+1));

				cloud->points = divisionPart(divisionNumber, input->points.begin(), input->points.end());

				cloud->width = (int) cloud->points.size();
				cloud->height = 1;
				return cloud;
			}

		private:
			bool pointIsInside(const PointT &point)
			{
				if(std::fabs(point.x*point.y*point.z) == 0.0){ return false; }	//remove all point at (0, 0, 0)

				double x = point.x - this->camera_offset;

				//pojection XYZ to 2D plane
				double X_Pojection = (point.x * this->camera_Depth) / point.z;
				double Y_Pojection = (point.y * this->camera_Depth) / point.z;

				if(X_Pojection < this->segmentation_Left_Bound) { return false; }
				if(X_Pojection > this->segmentation_Right_Bound) { return false; }
				if(Y_Pojection < this->segmentation_Up_Bound) { return false; }
				if(Y_Pojection > this->segmentation_Down_Bound) { return false; }

				return true;
			}

			template<typename RandomIt>
			std::vector<PointT, Eigen::aligned_allocator<PointT>> divisionPart(const int &division_num, const RandomIt &beg, const RandomIt &end)
			{
				auto len = end - beg;

				if (len < division_num)
				{
					std::vector<PointT, Eigen::aligned_allocator<PointT>> out;
					if(this->keep_Inside)
					{
						for(auto it = beg; it != end; ++it)
						{
							if(this->pointIsInside((*it)))
							{
								out.push_back(*it);
							}
						}
					}
					else
					{
						for(auto it = beg; it != end; ++it)
						{
							if(!this->pointIsInside((*it)))
							{
								out.push_back(*it);
							}
						}
					}
					return out;
				}

				auto mid = beg + len/2;
				auto handle = std::async(std::launch::async, &objectSegmentation::divisionPart<RandomIt>, this, division_num, beg, mid);
				auto out(divisionPart<RandomIt>(division_num, mid, end));
				auto out1(handle.get());
				
				std::copy(out1.begin(),  out1.end(), std::back_inserter(out));		//append out1 to out

				return out;
			}
	};
}

#endif