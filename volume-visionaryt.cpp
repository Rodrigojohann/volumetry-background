#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <termio.h>
#include <unistd.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "VisionaryTData.h"
#include "VisionaryDataStream.h"
#include "VisionaryControl.h"
#include "PointXYZ.h"
#include "PointCloudPlyWriter.h"
#include "VisionaryAutoIPScan.h"
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PassThrough<pcl::PointXYZ> passx;
pcl::PassThrough<pcl::PointXYZ> passy;
pcl::PassThrough<pcl::PointXYZ> passz;
pcl::ConvexHull<pcl::PointXYZ> chull;
pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
pcl::PointXYZ minPt, maxPt;
std::vector<pcl::Vertices> polygons;
double volume;
double volumemean;
std::vector<PointXYZ> pointCloud;
boost::shared_ptr<VisionaryTData> pDataHandler;
int counter;
int characters_buffered;
bool pressed;
struct termios original;
struct termios term;
size_t cloud_size;
double dimensionX;
double dimensionY;
double dimensionZ;

///////////////////////////////////////////////////////

bool kbhit(void)
{
    tcgetattr(STDIN_FILENO, &original);
    memcpy(&term, &original, sizeof(term));
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    characters_buffered = 0;
    ioctl(STDIN_FILENO, FIONREAD, &characters_buffered);
    tcsetattr(STDIN_FILENO, TCSANOW, &original);
    pressed = (characters_buffered != 0);
    return pressed;
}

double calculatevolume(std::vector<PointXYZ> inputcloud)
{
	cloud->points.resize (inputcloud.size());

	for(size_t i=0;i<cloud->points.size();++i)
	{
		cloud->points[i].x = inputcloud[i].x;
		cloud->points[i].y = inputcloud[i].y;
		cloud->points[i].z = inputcloud[i].z;
	}

	passx.setInputCloud (cloud);
	passx.setFilterFieldName ("x");
	passx.setFilterLimits (-0.23, 0.25);
	passx.filter (*cloud_filtered);
	
	passy.setInputCloud (cloud_filtered);
	passy.setFilterFieldName ("y");
	passy.setFilterLimits (-0.15, 0.26);
	passy.filter (*cloud_filtered);
	
	passz.setInputCloud (cloud_filtered);
	passz.setFilterFieldName ("z");
	passz.setFilterLimits (0, 0.758);
	passz.filter (*cloud_filtered);
	
	sor.setInputCloud (cloud_filtered);
	sor.setMeanK (5);
	sor.setStddevMulThresh (3.5);
	sor.filter (*cloud_filtered);

	cloud_size = cloud_filtered->size();
	
	if (cloud_size > 10)
	{
	chull.setInputCloud(cloud_filtered);
	chull.setDimension(3);
	chull.setComputeAreaVolume(true);
	chull.reconstruct(*surface_hull, polygons);
	
	volume = chull.getTotalVolume();

	pcl::getMinMax3D (*surface_hull, minPt, maxPt);
	dimensionX = maxPt.x - minPt.x;
	dimensionY = maxPt.y - minPt.y;
	dimensionZ = maxPt.z - minPt.z;
	
	}
	else
	{
	volume = 0.0;
	dimensionX = 0.0;
	dimensionY = 0.0;
	dimensionZ = 0.0;
	}
	return volume;
}

void runStreamingDemo(char* ipAddress, unsigned short port)
{
	// Generate Visionary instance
	pDataHandler = boost::make_shared<VisionaryTData>();
	VisionaryDataStream dataStream(pDataHandler, inet_addr(ipAddress), htons(port));
	VisionaryControl control(inet_addr(ipAddress), htons(2112));
	//-----------------------------------------------
	// Connect to devices data stream 
	if (!dataStream.openConnection())
	{
		printf("Failed to open data stream connection to device.\n");
	}
	//-----------------------------------------------
	// Connect to devices control channel
	if (!control.openConnection())
	{
		printf("Failed to open control connection to device.\n");
	}
	control.stopAcquisition();
	control.startAcquisition();
	
	volumemean = 0.0;
	while (!kbhit())
	{
		counter = counter+1; 
		if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
			// Calculate volume
			volumemean = volumemean + calculatevolume(pointCloud);
		}

		if (counter==9)
		{
			counter = 0;
			volumemean = volumemean/10;
			printf("---------------------\n\n");
			printf("volume: %f m³\n\n", volumemean);
			printf("volume: %f cm³\n\n", volumemean*1000000);
			
			printf("dimensions:\n");
			printf("%f cm (x)\n", dimensionX*100);
			printf("%f cm (y)\n", dimensionY*100);
			printf("%f cm (z)\n\n", dimensionZ*100);
			volumemean = 0.0;			
		}
	}
	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
}


int main()
{
	runStreamingDemo((char*)"192.168.140.29", 2114);
}
