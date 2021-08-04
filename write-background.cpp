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



std::vector<PointXYZ> pointCloud;
boost::shared_ptr<VisionaryTData> pDataHandler;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_concat (new pcl::PointCloud<pcl::PointXYZ>);
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
	
	for (int i = 0; i < 10; i++)
	{
	if (dataStream.getNextFrame())
		{
			// Convert data to a point cloud
			pDataHandler->generatePointCloud(pointCloud);
		}

	cloud->points.resize (pointCloud.size());

	for(size_t i=0;i<cloud->points.size();++i)
	{
		cloud->points[i].x = pointCloud[i].x;
		cloud->points[i].y = pointCloud[i].y;
		cloud->points[i].z = pointCloud[i].z;
	}
	
	cloud_concat += cloud;
	}
	
	char* plyFilePath = "backgroundcloud.ply";
	printf("Writing frame to %s\n", plyFilePath);
	PointCloudPlyWriter::WriteFormatPLY(plyFilePath, cloud_concat, true);
	printf("Finished writing frame to %s\n", plyFilePath);
	
	
	control.stopAcquisition();
	control.closeConnection();
	dataStream.closeConnection();
}


int main()
{
	runStreamingDemo((char*)"192.168.140.29", 2114);
}
