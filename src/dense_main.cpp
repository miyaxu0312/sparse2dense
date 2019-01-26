#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/point_types.h>                  //�����Ͷ���ͷ�ļ�
#include <pcl/point_cloud.h>                  //�����ඨ��ͷ�ļ�
#include <pcl/point_representation.h>         //����ʾ���ص�ͷ�ļ�
#include <fstream>
#include <vector>
#include<thread>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transforms.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <pcl/filters/extract_indices.h>
#include "opencv2/core/eigen.hpp"
#include "slamBase.h"
#include "XCTool.h"
using namespace std;

int main(int argc, char** argv)
{
	string para="/home/yixuan/DENSE/RGBDparameters.txt";
	ParameterReader pd;
	//cout<<argc<<endl;
	string camTransFile,associateFile,dataMainPath;
	/*
	if (argc > 1)
	{
		camTransFile= argv[1];
		associateFile = argv[2];
		dataMainPath = argv[3];
		//string RGBDparameters = argv[4];
		//ParameterReader pd(RGBDparameters);
	}
	
	else
	{*/
		camTransFile= "/home/yixuan/DENSE/desk2/groundtruth.txt";
		associateFile = "/home/yixuan/DENSE/fr1_desk2.txt";
		dataMainPath = "/home/yixuan/DENSE/desk2";
		//string RGBDparameters ;
		//ParameterReader pd;
	//}
	// �����ڲ�
	CAMERA_INTRINSIC_PARAMETERS camera;
	camera.fx = atof(pd.getData("camera.fx").c_str());
	camera.fy = atof(pd.getData("camera.fy").c_str());
	camera.cx = atof(pd.getData("camera.cx").c_str());
	camera.cy = atof(pd.getData("camera.cy").c_str());
	camera.scale = atof(pd.getData("camera.scale").c_str());
	cout << "line 45"<<endl;
	ifstream fcamTrans(camTransFile);
	vector<XCKey> keyVec;
	cout << "line 48" << endl;
	while (!fcamTrans.eof()) {
		XCKey tkey;
		fcamTrans >> tkey.frameID;
		fcamTrans >> tkey.tx;
		fcamTrans >> tkey.ty;
		fcamTrans >> tkey.tz;
		fcamTrans >> tkey.qx;
		fcamTrans >> tkey.qy;
		fcamTrans >> tkey.qz;
		fcamTrans >> tkey.qw;
		keyVec.push_back(tkey);
		//cout << "*" << endl;
	}
	cout<<keyVec.size()<<endl;
	cout << "line 60" << endl;
	ifstream fAssociation(associateFile);
	vector<XCAssociationKey> assoKeyVec;
	while (!fAssociation.eof()) {
		XCAssociationKey takey;
		fAssociation >> takey.rgb;
		fAssociation >> takey.full_rgb;
		fAssociation >> takey.d;
		fAssociation >> takey.full_d;
		assoKeyVec.push_back(takey);
	}
	cout<<assoKeyVec.size()<<endl;
	cout << "line 71" << endl;
	vector<string> rgbPathVec, dPathVec;
	for (int i = 0; i < assoKeyVec.size(); i++) {
		rgbPathVec.push_back(dataMainPath + "/rgb/" + assoKeyVec[i].rgb + ".png");
		//cout << dataMainPath <<"/rgb/"<<assoKeyVec[i].rgb<< endl;
		dPathVec.push_back(dataMainPath + "/depth/" + FindDFileByRGB(assoKeyVec, assoKeyVec[i].rgb) + ".png");
		//cout << "line 79" << endl;
	}

	cout << "Initing...\n";
	//��ʼ������
	vector<PointCloud::Ptr> pcVec;
	for (int i = 0; i < assoKeyVec.size(); i++) {
		FRAME tframe;
		tframe.rgb = cv::imread(rgbPathVec[i]);
		tframe.depth = cv::imread(dPathVec[i], -1);
		//cout<<rgbPathVec[i]<<endl;
		pcVec.push_back(image2PointCloud(tframe.rgb, tframe.depth, camera));
		//cout << i << "/" << rgbPathVec.size() <<endl;
	}

	cout << "Filtering...\n";
	//�˲�
	bool bFilter = true;
	int i = 1;
	if (bFilter) {
		for (auto& pciter : pcVec) {
			PointCloud::Ptr cloud_filtered(new PointCloud());
			pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
			sor.setInputCloud(pciter);
			sor.setLeafSize(0.01f, 0.01f, 0.01f);
			sor.filter(*cloud_filtered);
			pciter = cloud_filtered;

			PointCloud::Ptr cloud_filtered2(new PointCloud());
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> sor2;
			sor2.setInputCloud(cloud_filtered);
			sor2.setMeanK(50);
			sor2.setStddevMulThresh(1.0);
			sor2.filter(*cloud_filtered2);
			//cout << i << "/" << pcVec.size() << endl;
			i++;
			pciter = cloud_filtered2;
		}
	}
	for (int i = 0; i<pcVec.size(); i++){
		double x = keyVec[i].qx, y = keyVec[i].qy, z = keyVec[i].qz, w = keyVec[i].qw;

		cv::Mat R;

		R = (cv::Mat_<double>(3, 3) <<
			2 * (x*x + w*w) - 1, 2 * (x*y + z*w), 2 * (x*z - y*w),
			2 * (x*y - z*w), 2 * (y*y + w*w) - 1, 2 * (y*z + x*w),
			2 * (x*z + y*w), 2 * (y*z - x*w), 2 * (z*z + w*w) - 1
			);
		R = R.inv();
		Eigen::Matrix3d r;
		cv::cv2eigen(R, r);

		// ��ƽ����������ת����ת���ɱ任����
		Eigen::Isometry3d T = Eigen::Isometry3d::Identity();

		Eigen::AngleAxisd angle(r);
		//cout << "translation" << endl;

		T = angle;
		T(0, 3) = keyVec[i].tx;
		T(1, 3) = keyVec[i].ty;
		T(2, 3) = keyVec[i].tz;

		PointCloud::Ptr toutput(new PointCloud());
		pcl::transformPointCloud(*pcVec[i], *toutput, T.matrix());
		pcVec[i] = toutput;
		//cout << i << "/" << pcVec.size() << endl;
	}
	cout << "trans over\n";
	PointCloud::Ptr allOutput(new PointCloud()), allAfterFilter(new PointCloud());
	int algoFlag = 0;
	bool bNDT = false;
	int ndtindex = 0;
	bool bPCH = false;
	if (algoFlag == 0) {
		for (int i = 0; i < pcVec.size(); i++) {
			*allOutput += *pcVec[i];
		}
	}
	/*
	else if (algoFlag == 1) {
		allOutput = pcVec[0];
		for (int i = 0; i < pcVec.size() - 1; i++) {
			PairwiseICP(pcVec[i + 1], allOutput, allOutput);
		}
	}
	else if (algoFlag == 2){
		//��¼ÿ������������ICP�ľ���
		vector<Eigen::Matrix4f> icpMatVec;
		vector<function<void(void)>> funcVec;
		vector<thread> taskVec;
		for (int i = 0; i < pcVec.size() - 1; i++) {
			if (bPCH) {
				funcVec.push_back([&]() {icpMatVec.push_back(XCPairwisePCH(pcVec[i], pcVec[i + 1])); });
			}
			else if (bNDT&&i == ndtindex){
				funcVec.push_back([&]() {icpMatVec.push_back(XCPairwiseNDT(pcVec[i], pcVec[i + 1])); });
			}
			else {
				funcVec.push_back([&]() {
					icpMatVec.push_back(XCPairwiseICP(pcVec[i], pcVec[i + 1]));
					/*double t1 = icpMatVec[i](0, 3);
					double t2 = icpMatVec[i](1, 3);
					double t3 = icpMatVec[i](2, 3);
					double r1 = icpMatVec[i](0, 0);
					double r2 = icpMatVec[i](0, 1);
					double r3 = icpMatVec[i](0, 2);
					double r4 = icpMatVec[i](1, 0);
					double r5 = icpMatVec[i](1, 1);
					double r6 = icpMatVec[i](1, 2);
					double r7 = icpMatVec[i](2, 0);
					double r8 = icpMatVec[i](2, 1);
					double r9 = icpMatVec[i](2, 2);
					cout << i+1<<"-"<<i+2<<endl;
					cout << "t1:" << t1 << " t2:" << t2 << " t3:" << t3<<endl;
					cout << "t��" << sqrt(t1*t1 + t2*t2 + t3*t3)<<endl;
					cout << "R:\n";
					cout << r1 << " " << r2 << " " << r3 << endl;
					cout << r4 << " " << r5 << " " << r6 << endl;
					cout << r7 << " " << r8 << " " << r9 << endl;
				});
			}
			taskVec.push_back(thread(funcVec[i]));
			taskVec[i].join();
		}
		cout << "PairOver\n";
		//ͬ��
		for (int i = 1; i < pcVec.size(); i++) {
			for (int i2 = 0; i2 < i; i2++) {
				PointCloud::Ptr toutput(new PointCloud());
				pcl::transformPointCloud(*pcVec[i2], *toutput, icpMatVec[i - 1]);
				pcVec[i2] = toutput;
				cout << "@fusion:" << i2 + 1 << endl;
			}
		}

		for (int i = 0; i < pcVec.size(); i++) {
			*allOutput += *pcVec[i];
		}
	}
	*/
	pcl::io::savePCDFile("result.pcd", *allOutput);
	cout << "Final result saved." << endl;
	/*
	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(allOutput);
	while (!viewer.wasStopped())
	{

	}
	*/
	return (0);
}
