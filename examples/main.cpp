#include <iostream>
//#include <pcl/io/pcd_io.h>
#include "TINConstruct.h"
#include "tree_point.h"
#include "descriptor.h"
#include "match.h"
#include <pcl/io/pcd_io.h>
// spdlog
#include <spdlog/spdlog.h>
#include <string>
#include <chrono>

int main()
{
	std::string workspace = "F:\\dianzhong\\";//
	std::vector<int> n{5};//dianzhong

//	std::string workspace = "F:\\xianggelila\\";//1为30,10；2为30,30;6为100,200;7为30,30;9为30,30;10为30,10;
//	std::vector<int> n{7};//xianggelila

//	std::string workspace = "F:\\saihanba\\";//1-100,30;   2、3、4-30,30;
//	std::vector<int> n{ 2,3,4 };//saihanba

	//记录时间表格CSV
	std::string time_file=workspace+"Time_0327.csv";
	std::ofstream outfile(time_file);
	outfile << "number,extractPoint,TINConstruct,descriptor,match,ransac_svd,time_all" << std::endl;

	for (int q = 0; q < n.size(); q++)
	{
		int number = n[q];
		std::cout << "Start: " << number << std::endl;
		std::string filename_src = workspace + std::to_string(number) + "\\TLS_norm.pcd";
		std::string filename_tgt = workspace + std::to_string(number) + "\\ULS_norm.pcd";

		//------------------------------- 0.数据读取 -------------------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_src(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
		pcl::io::loadPCDFile<pcl::PointXYZ>(filename_src, *input_cloud_src);
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
		pcl::io::loadPCDFile<pcl::PointXYZ>(filename_tgt, *input_cloud_tgt);

		input_cloud_src->width = input_cloud_src->size();
		input_cloud_src->height = 1;
		input_cloud_tgt->width = input_cloud_tgt->size();
		input_cloud_tgt->height = 1;
		spdlog::info("point cloud import success! input_cloud_src:{}", input_cloud_src->size());
		spdlog::info("point cloud import success! input_cloud_tgt:{}", input_cloud_tgt->size());


		//------------------------------- 1.树顶点提取 -------------------------------
		auto time_start_extractPoint = std::chrono::high_resolution_clock::now();

		PointExtraction PtExtract;
		int k1 = 30;//邻域参数
		int k2 = 30;
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_src(new pcl::PointCloud<pcl::PointXYZ>);//中间生成的点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr seed_cloud_src(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);//中间生成的点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr seed_cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
		PtExtract.Extract_Point(input_cloud_src, temp_cloud_src, k1);
		PtExtract.Extract_Point(input_cloud_tgt, temp_cloud_tgt, k1);
		pcl::io::savePCDFile(workspace + std::to_string(number) + "\\TLS_temp_seed.pcd", *temp_cloud_src);
		pcl::io::savePCDFile(workspace + std::to_string(number) + "\\ULS_temp_seed.pcd", *temp_cloud_tgt);
//		pcl::io::loadPCDFile<pcl::PointXYZ>(workspace + std::to_string(number) + "\\TLS_temp_seed.pcd", *temp_cloud_src);
//		pcl::io::loadPCDFile<pcl::PointXYZ>(workspace + std::to_string(number) + "\\ULS_temp_seed.pcd", *temp_cloud_tgt);

		auto time_start_extractPoint_1 = std::chrono::high_resolution_clock::now();
		auto duration_extractPoint_cu = std::chrono::duration_cast<std::chrono::microseconds>(time_start_extractPoint_1 - time_start_extractPoint);
		PtExtract.extract_fine(input_cloud_src, temp_cloud_src, k2, seed_cloud_src);
		PtExtract.extract_fine(input_cloud_tgt, temp_cloud_tgt, k2, seed_cloud_tgt);
		pcl::io::savePCDFile(workspace + std::to_string(number) + "\\TLS_seed.pcd", *seed_cloud_src);
		pcl::io::savePCDFile(workspace + std::to_string(number) + "\\ULS_seed.pcd", *seed_cloud_tgt);

		auto time_end_extractPoint = std::chrono::high_resolution_clock::now();
		auto duration_extractPoint = std::chrono::duration_cast<std::chrono::microseconds>(time_end_extractPoint - time_start_extractPoint);

		auto duration_extractPoint_xi = std::chrono::duration_cast<std::chrono::microseconds>(time_end_extractPoint - time_start_extractPoint_1);
//		spdlog::info("PointExtraction Time: {}  seconds", duration_extractPoint.count()*0.000001);

//	PtExtract.Extract_Point(temp_cloud_src, seed_cloud_src, k2);
//	PtExtract.Extract_Point(temp_cloud_tgt, seed_cloud_tgt, k2);
		if (seed_cloud_src->empty() || seed_cloud_tgt->empty())
		{
			spdlog::error("point extraction empty");
			return 0;
		}
		spdlog::info("point extraction finished! seed_cloud_src size: {}", seed_cloud_src->size());
		spdlog::info("point extraction finished! seed_cloud_tgt size: {}", seed_cloud_tgt->size());
//		pcl::io::savePCDFile("F:\\xianggelila\\07_XGLL20210818001\\TLS_seed.pcd", *seed_cloud_src);
//		pcl::io::savePCDFile("F:\\xianggelila\\07_XGLL20210818001\\ULS_seed.pcd", *seed_cloud_tgt);


		//------------------------------- 2.构建TIN提取出边 -------------------------------
		auto time_start_TINConstruct = std::chrono::high_resolution_clock::now();
		TINConstruct TC;
		std::vector<Line> line_all_src;
		std::vector<Line> line_all_tgt;
		TC.construct(seed_cloud_src, line_all_src);
		TC.construct(seed_cloud_tgt, line_all_tgt);

//		KDNetwork KN;
//		std::vector<Line> line_all_src;
//		std::vector<Line> line_all_tgt;
//		KN.buildNetwork(seed_cloud_src, line_all_src);
//		KN.buildNetwork(seed_cloud_tgt, line_all_tgt);

		auto time_end_TINConstruct = std::chrono::high_resolution_clock::now();
		auto duration_TINConstruct = std::chrono::duration_cast<std::chrono::microseconds>(time_end_TINConstruct - time_start_TINConstruct);
//		spdlog::info("TINConstruct Time: {}  seconds", duration_TINConstruct.count()*0.000001);

		if (line_all_src.empty() || line_all_tgt.empty())
		{
			spdlog::error("the size of line is empty");
			return 0;
		}
		spdlog::info("TIN construct finished! line_all_src: {} line_all_tgt: {} ", line_all_src.size(), line_all_tgt.size());


		//---------------------------------- 3.构建描述子 ----------------------------------
		//3.对边构建描述子，角度，长度，高度_较低，高度_较高
		auto time_start_descriptor = std::chrono::high_resolution_clock::now();

		float grid_size = 2.0;
		getDescriptor(line_all_src, seed_cloud_src, input_cloud_src, grid_size);
		getDescriptor(line_all_tgt, seed_cloud_tgt, input_cloud_tgt, grid_size);

		auto time_end_descriptor = std::chrono::high_resolution_clock::now();
		auto duration_descriptor = std::chrono::duration_cast<std::chrono::microseconds>(time_end_descriptor - time_start_descriptor);
//		spdlog::info("getDescriptor Time: {}  seconds", duration_TINConstruct.count()*0.000001);
		spdlog::info("descriptors finished");


		//---------------------------- 4.获取描述子的最邻近匹配线对 ----------------------------
		auto time_start_match = std::chrono::high_resolution_clock::now();
		std::vector<std::vector<float>> feature_src;
		for (int i = 0; i < line_all_src.size(); i++)
		{
			std::vector<float> des_src;
			des_src.push_back(line_all_src[i].des.angle);
			des_src.push_back(line_all_src[i].des.length);
			des_src.push_back(line_all_src[i].des.height_low);
			des_src.push_back(line_all_src[i].des.height_tall);
			feature_src.push_back(des_src);
		}
		std::vector<std::vector<float>> feature_tgt;
		for (int i = 0; i < line_all_tgt.size(); i++)
		{
			std::vector<float> des_tgt;
			des_tgt.push_back(line_all_tgt[i].des.angle);
			des_tgt.push_back(line_all_tgt[i].des.length);
			des_tgt.push_back(line_all_tgt[i].des.height_low);
			des_tgt.push_back(line_all_tgt[i].des.height_tall);
			feature_tgt.push_back(des_tgt);
		}
		std::vector<std::pair<int, int>> corres;
		std::vector<float> rmse_all;
		MatchPair(feature_src, feature_tgt, corres, rmse_all);
		spdlog::info("get corres success!corres size:{}", corres.size());

		auto time_end_match = std::chrono::high_resolution_clock::now();
		auto duration_match = std::chrono::duration_cast<std::chrono::microseconds>(time_end_match - time_start_match);


		//--------------------------------- 5.获取旋转变换矩阵 ---------------------------------
		auto time_start_ransac_svd = std::chrono::high_resolution_clock::now();
		std::vector<std::pair<int, int>> corres_point;
		for (int i = 0; i < corres.size(); i++)
		{
			corres_point.push_back(std::pair<int, int>(line_all_src[corres[i].first].idx1, line_all_tgt[corres[i].second].idx1));
			corres_point.push_back(std::pair<int, int>(line_all_src[corres[i].first].idx2, line_all_tgt[corres[i].second].idx2));

		}
		Eigen::Matrix4f RT;//变换矩阵
		int iterations = 100000;//迭代次数
		float dist_thresh = 0.8;//阈值
		if (RT_RANSAC3(seed_cloud_src, seed_cloud_tgt, temp_cloud_src, temp_cloud_tgt, corres_point, iterations, dist_thresh, RT))
		{
			spdlog::info("transform matrix calculate finish!");
			std::cout << "transform matrix: \n" << RT << std::endl;
		}

//		if (RT_RANSAC(seed_cloud_src, seed_cloud_tgt, corres_point, iterations, dist_thresh, RT))
//		{
//			spdlog::info("transform matrix calculate finish!");
//			std::cout << "transform matrix: \n" << RT << std::endl;
//		}

		auto time_end_ransac_svd = std::chrono::high_resolution_clock::now();
		auto duration_ransac_svd = std::chrono::duration_cast<std::chrono::microseconds>(time_end_ransac_svd - time_start_ransac_svd);

		std::string TXTfile = workspace + std::to_string(number) + "\\cal_RT_0527.txt";
		std::ofstream outFile(TXTfile);
		if (outFile.is_open())
		{
			outFile << RT;
			outFile.close();
			spdlog::info("Matrix successfully saved to cal_RT.txt.");
		}

		spdlog::info("Point Extract Time: {}  seconds", duration_extractPoint.count() * 0.000001);
		spdlog::info("Point Extract Time 粗提取: {}  seconds", duration_extractPoint_cu.count() * 0.000001);
		spdlog::info("Point Extract Time 细提取: {}  seconds", duration_extractPoint_xi.count() * 0.000001);
		spdlog::info("TIN Construct Time: {}  seconds", duration_TINConstruct.count() * 0.000001);
		spdlog::info("Get Descriptor Time: {}  seconds", duration_descriptor.count() * 0.000001);
		spdlog::info("RANSAC Calculate Matrix Time: {}  seconds", duration_ransac_svd.count() * 0.000001);
		float time_all = (duration_extractPoint.count() + duration_TINConstruct.count()
			+ duration_descriptor.count() + duration_ransac_svd.count()) * 0.000001;
		spdlog::info("Total time: {}  seconds", time_all);

		outfile << number << "," << duration_extractPoint.count() * 0.000001 << ","
				<< duration_TINConstruct.count() * 0.000001 << ","
				<< duration_descriptor.count() * 0.000001 << ","
				<< duration_match.count()*0.000001<<","
				<< duration_ransac_svd.count() * 0.000001 << ","
				<< time_all << std::endl;
	}
	outfile.close();
	std::cout << "finished !" << std::endl;

	return 0;
}
