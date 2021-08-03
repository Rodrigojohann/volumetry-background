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
	
	if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
		}

	char* plyFilePath = "backgroundcloud.ply";
	printf("Writing frame to %s\n", plyFilePath);
	PointCloudPlyWriter::WriteFormatPLY(plyFilePath, pointCloud, true);
	printf("Finished writing frame to %s\n", plyFilePath);
	
	}
	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
}


int main()
{
	runStreamingDemo((char*)"192.168.140.29", 2114);
}
