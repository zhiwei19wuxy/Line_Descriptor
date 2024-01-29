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

int main()
{
//	std::string filename_src = "F:\\xianggelila\\01_XGLL_2020092301\\TLS_1.pcd";//精度挺高的，k,30,20
//	std::string filename_tgt = "F:\\xianggelila\\01_XGLL_2020092301\\ULS_1.pcd";

//	std::string filename_src = "D:\\data\\registration_test\\TLS-20200924001-Cloud2.pcd";//可以得到结果
//	std::string filename_tgt = "D:\\data\\registration_test\\ULS-20210819001-Cloud.pcd";

//	std::string filename_src = "F:\\xianggelila\\03_XGLL-2020092402\\TLS-20200924002-Cloud - Cloud.pcd";//只有四对匹配的
//	std::string filename_tgt = "F:\\xianggelila\\03_XGLL-2020092402\\ULS-20210819002-Cloud - Cloud.pcd";//配准之前匹配度很高
//
//	std::string filename_src = "F:\\xianggelila\\04_XGLL20200917001\\TLS.pcd";//有warning,但矩阵输出后可以配准
//	std::string filename_tgt = "F:\\xianggelila\\04_XGLL20200917001\\ULS.pcd";

//	std::string filename_src = "F:\\xianggelila\\05_XGLL20200918001\\TLS.pcd";//可以配上，k2=5
//	std::string filename_tgt = "F:\\xianggelila\\05_XGLL20200918001\\ULS.pcd";

//	std::string filename_src = "F:\\xianggelila\\06_XGLL20210816001\\TLS.pcd";//可以配上，
//	std::string filename_tgt = "F:\\xianggelila\\06_XGLL20210816001\\ULS.pcd";

	std::string filename_src = "F:\\xianggelila\\07_XGLL20210818001\\TLS.pcd";//k1=30 k2=30可以配上
	std::string filename_tgt = "F:\\xianggelila\\07_XGLL20210818001\\ULS.pcd";

//	std::string filename_src = "F:\\xianggelila\\08_XGLL20210818002\\TLS.pcd";//配的上
//	std::string filename_tgt = "F:\\xianggelila\\08_XGLL20210818002\\ULS.pcd";

//	std::string filename_src = "F:\\xianggelila\\09_XGLL20210818003\\TLS.pcd";//
//	std::string filename_tgt = "F:\\xianggelila\\09_XGLL20210818003\\ULS.pcd";

//	std::string filename_src = "F:\\xianggelila\\11_XGLL20210821001\\TLS.pcd";//
//	std::string filename_tgt = "F:\\xianggelila\\11_XGLL20210821001\\ULS.pcd";

//	std::string filename_src = "F:\\saihanba\\bh\\TLS.pcd";//可以
//	std::string filename_tgt = "F:\\saihanba\\bh\\ULS.pcd";

//	std::string filename_src = "F:\\saihanba\\hjl\\TLS.pcd";//可以配上
//	std::string filename_tgt = "F:\\saihanba\\hjl\\ULS.pcd";

//	std::string filename_src = "F:\\saihanba\\lys\\TLS.pcd";//偏差很多
//	std::string filename_tgt = "F:\\saihanba\\lys\\ULS.pcd";

//	std::string filename_src = "F:\\saihanba\\zzs_adult\\TLS.pcd";//
//	std::string filename_tgt = "F:\\saihanba\\zzs_adult\\ULS.pcd";




	//数据读取
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_src(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
	pcl::io::loadPCDFile<pcl::PointXYZ>(filename_src, *input_cloud_src);

	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
	pcl::io::loadPCDFile<pcl::PointXYZ>(filename_tgt, *input_cloud_tgt);

	input_cloud_src->width = input_cloud_src->size();
	input_cloud_src->height = 1;
	input_cloud_tgt->width = input_cloud_tgt->size();
	input_cloud_tgt->height = 1;
//	pcl::io::savePCDFile("D:\\data\\registration_test\\TLS-20200924001-Cloud2_seed.pcd", *input_cloud_src);
//	pcl::io::savePCDFile("D:\\data\\registration_test\\ULS-20210819001-Cloud_seed.pcd", *input_cloud_tgt);
	spdlog::info("point cloud import success! input_cloud_src:{}", input_cloud_src->size());
	spdlog::info("point cloud import success! input_cloud_tgt:{}", input_cloud_tgt->size());




	//0.树顶点提取
	PointExtraction PtExtract;
	int k1 = 30;//邻域参数
	int k2 = 20;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_src(new pcl::PointCloud<pcl::PointXYZ>);//中间生成的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr seed_cloud_src(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);//中间生成的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr seed_cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);//原始点云
//	PtExtract.Extract_Point(input_cloud_src, temp_cloud_src, k1);
//	PtExtract.Extract_Point(input_cloud_tgt, temp_cloud_tgt, k1);
//	pcl::io::savePCDFile("F:\\xianggelila\\07_XGLL20210818001\\TLS_temp.pcd", *temp_cloud_src);
//	pcl::io::savePCDFile("F:\\xianggelila\\07_XGLL20210818001\\ULS_temp.pcd", *temp_cloud_tgt);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:\\xianggelila\\07_XGLL20210818001\\TLS_temp.pcd", *temp_cloud_src);
	pcl::io::loadPCDFile<pcl::PointXYZ>("F:\\xianggelila\\07_XGLL20210818001\\ULS_temp.pcd", *temp_cloud_tgt);
	PtExtract.extract_fine(input_cloud_src,temp_cloud_src, k2,seed_cloud_src);
	PtExtract.extract_fine(input_cloud_tgt,temp_cloud_tgt, k2,seed_cloud_tgt);
//	PtExtract.Extract_Point(temp_cloud_src, seed_cloud_src, k2);
//	PtExtract.Extract_Point(temp_cloud_tgt, seed_cloud_tgt, k2);
	if (seed_cloud_src->empty() || seed_cloud_tgt->empty())
	{
		spdlog::error("point extraction empty");
		return 0;
	}
	spdlog::info("point extraction finished! seed_cloud_src size: {}", seed_cloud_src->size());
	spdlog::info("point extraction finished! seed_cloud_tgt size: {}", seed_cloud_tgt->size());
//	pcl::io::savePCDFile("F:\\xianggelila\\07_XGLL20210818001\\TLS_seed.pcd", *seed_cloud_src);
//	pcl::io::savePCDFile("F:\\xianggelila\\07_XGLL20210818001\\ULS_seed.pcd", *seed_cloud_tgt);
//


	//1.构建TIN提取出边
	TINConstruct TC;
	std::vector<Line> line_all_src;
	std::vector<Line> line_all_tgt;
	TC.construct(seed_cloud_src, line_all_src);
	TC.construct(seed_cloud_tgt, line_all_tgt);
	if (line_all_src.empty() || line_all_tgt.empty())
	{
		spdlog::error("the size of line is empty");
		return 0;
	}
	spdlog::info("TIN construct finished! line_all_src: {} line_all_tgt: {} ", line_all_src.size(), line_all_tgt.size());


	//2.对边构建描述子，四个属性，角度，长度，高度_较低，高度_较高
	float grid_size = 2.0;
	getDescriptor(line_all_src, seed_cloud_src, input_cloud_src, grid_size);
	getDescriptor(line_all_tgt, seed_cloud_tgt, input_cloud_tgt, grid_size);
	spdlog::info("descriptors finished");


	//3.获取描述子的最邻近匹配线对
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


	//4.获取旋转变换矩阵
	std::vector<std::pair<int, int>> corres_point;
	for (int i = 0; i < corres.size(); i++)
	{
		corres_point.push_back(std::pair<int, int>(line_all_src[corres[i].first].idx1, line_all_tgt[corres[i].second].idx1));
		corres_point.push_back(std::pair<int, int>(line_all_src[corres[i].first].idx2, line_all_tgt[corres[i].second].idx2));

	}
	Eigen::Matrix4f RT;//变换矩阵
	int iterations = 10000;//迭代次数
	float dist_thresh = 0.8;//阈值
	if (RT_RANSAC2(seed_cloud_src, seed_cloud_tgt,temp_cloud_src,temp_cloud_tgt, corres_point, iterations, dist_thresh, RT))
	{
		spdlog::info("transform matrix calculate finish!");
		std::cout << "transform matrix: \n" << RT << std::endl;
	}

	return 0;
}
