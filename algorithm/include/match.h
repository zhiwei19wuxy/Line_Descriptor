/**
 *  \file   match.h
 *  \brief
 *  \author wxy
 *  \email  wuxiaoyang22@mails.ucas.ac.cn
 *  \date   2023/12/22
 *  \note   
 */
//
#pragma once
#include <pcl/search/flann_search.h>
#include <vector>
typedef flann::Index<flann::L2<float>> KDTree;

// 建立特征的KDtree
void BuildKDTree(const std::vector<std::vector<float>>& feature, KDTree* tree);

// 搜索特征最近邻
void SearchKDTree(KDTree* tree, const std::vector<float>& feature, std::vector<int>& indices, std::vector<float>& dists, int nn);

// 匹配
bool MatchPair(const std::vector<std::vector<float>>& feature_src,
	const std::vector<std::vector<float>>& feature_tgt,
	std::vector<std::pair<int, int>>& corres,
	std::vector<float>& feature_rmse);

bool RT_RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_tgt,
	const std::vector<std::pair<int, int>>& corres,
	int k, float dist_thresh, Eigen::Matrix4f& rt);

bool RT_RANSAC2(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_tgt,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_temp_src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_temp_tgt,
	const std::vector<std::pair<int, int>>& corres,
	int k, float dist_thresh, Eigen::Matrix4f& rt);

struct SC2_Params
{
	float dthr;                   // 二值化SC矩阵时用于判断是否相似的阈值
	float pointThre;              // 计算corres中点变换是否正确的阈值
	float seedsRatio;             // 种子点选择比例
	float radius;                 // 种子点采样时，在radius范围内只采样一个点，非极大线性抑制
	int k1;                       // 每个种子点扩充时第一阶段选择corres个数
	int k2;                       // 扩充时第二阶段选择corres个数
	int leadVecIter;              // power iteration算法迭代次数

	SC2_Params()
	{
		dthr = 0.1;
		pointThre = 0.3;
		seedsRatio = 0.2;
		radius = 0.2;
		k1 = 30;
		k2 = 20;
		leadVecIter = 20;
	}
};

bool RT_SC2(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_tgt,
	const std::vector<std::pair<int, int>>& corres,
	SC2_Params params, Eigen::Matrix4f& rt);
