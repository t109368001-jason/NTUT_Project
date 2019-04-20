#include <basic_function.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef boost::shared_ptr<PointCloudT> PointCloudPtrT;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtrT;

namespace boost{namespace filesystem{

    boost::filesystem::path relative(boost::filesystem::path from, boost::filesystem::path to)
    {
    // Start at the root path and while they are the same then do nothing then when they first
    // diverge take the entire from path, swap it with '..' segments, and then append the remainder of the to path.
    boost::filesystem::path::const_iterator fromIter = from.begin();
    boost::filesystem::path::const_iterator toIter = to.begin();

    // Loop through both while they are the same to find nearest common directory
    while (fromIter != from.end() && toIter != to.end() && (*toIter) == (*fromIter))
    {
        ++toIter;
        ++fromIter;
    }

    // Replace from path segments with '..' (from => nearest common directory)
    boost::filesystem::path finalPath;
    while (fromIter != from.end())
    {
        finalPath /= "..";
        ++fromIter;
    }

    // Append the remainder of the to path (nearest common directory => to)
    while (toIter != to.end())
    {
        finalPath /= *toIter;
        ++toIter;
    }

    return finalPath;
    }
}}

namespace myFunction
{
	bool fileExists(const std::string &filename)
	{
		struct stat buffer;
		return (stat(filename.c_str(), &buffer) == 0);
	}

    float distance(const pcl::PointXYZ &point1, const pcl::PointXYZ &point2) {
        std::sqrt((point1.x-point2.x)*(point1.x-point2.x)+(point1.y-point2.y)*(point1.y-point2.y)+(point1.z-point2.z)*(point1.z-point2.z));
    }

	void valueToRGB(uint8_t &r, uint8_t &g, uint8_t &b, const float &value)
	{
        float tmp = value;
        tmp *= 1280.0;
        tmp += 128.0;
        if(tmp >= 0 && tmp < 256) {
			r = 0;
			g = 0;
			b = tmp;
        } else if(tmp >= 256 && tmp < 512) {
			r = 0;
			g = tmp - 255;
			b = 255;
        } else if(tmp >= 512 && tmp < 768) {
			r = 0;
			g = 255;
			b = 767-tmp;
        } else if(tmp >= 768 && tmp < 1024) {
			r = tmp - 767;
			g = 255;
			b = 0;
        } else if(tmp >= 1024 && tmp < 1280) {
			r = 255;
			g = 1279-tmp;
			b = 0;
        } else {
			r = 1535-tmp;
			g = 0;
			b = 0;
        }
	}

	void valueToRGB(uint32_t &rgb, const float &value)
	{
        float tmp = value;
        uint8_t r, g, b;
        tmp *= 1280.0;
        tmp += 128.0;
        if(tmp >= 0 && tmp < 256) {
			r = 0;
			g = 0;
			b = tmp;
        } else if(tmp >= 256 && tmp < 512) {
			r = 0;
			g = tmp - 255;
			b = 255;
        } else if(tmp >= 512 && tmp < 768) {
			r = 0;
			g = 255;
			b = 767-tmp;
        } else if(tmp >= 768 && tmp < 1024) {
			r = tmp - 767;
			g = 255;
			b = 0;
        } else if(tmp >= 1024 && tmp < 1280) {
			r = 255;
			g = 1279-tmp;
			b = 0;
        } else {
			r = 1535-tmp;
			g = 0;
			b = 0;
        }
        rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	}

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> XYZ_to_XYZRGB(const PointCloudPtrT &cloud, const float maxRange) {
        std::vector<float> range = getPointCloudRange<decltype(cloud->points.begin())>(std::log2(std::thread::hardware_concurrency()+1), cloud->points.begin(), cloud->points.end());
        
        range[1] = std::fmin(range[1], maxRange);

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        cloud_rgb->points = XYZ_to_XYZRGBPart<decltype(cloud->points.begin())>(std::log2(std::thread::hardware_concurrency()+1), cloud->points.begin(), cloud->points.end(), range);
        cloud_rgb->width = static_cast<uint32_t>(cloud_rgb->points.size());
		cloud_rgb->height = 1;

		return cloud_rgb;
    }

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const uint8_t &r, const uint8_t &g, const uint8_t &b) {

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, r, g, b);

		if( !viewer->updatePointCloud<pcl::PointXYZ> (cloud, rgb, name))
		{
			viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, name);
		}
    }

    void updateCloud(ViewerPtrT &viewer, const PointCloudPtrT &cloud, const std::string &name, const double maxRange) {

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_rgb;
        cloud_rgb = XYZ_to_XYZRGB(cloud, maxRange);

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);

		if( !viewer->updatePointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, name))
		{
			viewer->addPointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, name);
		}
    }
}
