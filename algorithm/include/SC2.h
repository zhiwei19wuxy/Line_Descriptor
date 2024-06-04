/**
 *  \file   SC2.h
 *  \brief
 *  \author wxy
 *  \email  wuxiaoyang22@mails.ucas.ac.cn
 *  \date   2023/12/24
 *  \note   
 */
//

#pragma once
// system
#include <vector>
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class SC2
{
 private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr sour;    // 源点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr tar;     // 目标点云
	Eigen::Matrix4f transMatrix;                 // transMatrix*sour=tar
	Eigen::MatrixXf scMat;                       // 一阶兼容矩阵
	Eigen::MatrixXd seedHardMat;                 // 种子点二值化SC矩阵
	Eigen::MatrixXd sc2Mat;                      // 二阶兼容矩阵
	Eigen::MatrixXf weights;
	std::vector<std::pair<int, int>> corres;     // 存储匹配的特征对，值表示点云序列号
	std::vector<int> seeds;                      // 存储种子点 其值指在scMat中的序列号

	float dthr;                   // 二值化SC矩阵时用于判断是否相似的阈值
	float pointThre;              // 计算corres中点变换是否正确的阈值
	float seedsRatio;             // 种子点选择比例
	float radius;                 // 种子点采样时，在radius范围内只采样一个点，非极大线性抑制
	int k1;                       // 每个种子点扩充时第一阶段选择corres个数
	int k2;                       // 扩充时第二阶段选择corres个数
	int leadVecIter;              // power iteration算法迭代次数

	void pickSeeds();             // 选择种子点
	void calScMat();
	void calScHardMat();
	void calSc2Mat();
	void calWeight(std::vector<int>& Col);
	Eigen::Matrix4f calBestTrans();
	Eigen::Matrix4f calTrans(const int index);

 public:
	SC2();
	~SC2();
	void setcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr sour_, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_);
	void setcorres(std::vector<std::pair<int, int>> corres_)
	{
		corres = corres_;
	}
	// 二值化SC矩阵时用于判断是否相似的阈值
	void setdthr(float dthr_)
	{
		dthr = dthr_;
	}
	// 计算corres中点变换是否正确的阈值
	void setpointThre(float pointThre_)
	{
		pointThre = pointThre_;
	}
	// 种子点选择比例
	void setseedsRatio(float seedsRatio_)
	{
		seedsRatio = seedsRatio_;
	}
	// 种子点采样时，在radius范围内只采样一个点，非极大线性抑制
	void setradius(float radius_)
	{
		radius = radius_;
	}
	// 每个种子点扩充时第一阶段选择corres个数
	void setk1(float k1_)
	{
		k1 = k1_;
	}
	// 扩充时第二阶段选择corres个数
	void setk2(float k2_)
	{
		k2 = k2_;
	}
	// power iteration算法迭代次数
	void setleadVecIter(float leadVecIter_)
	{
		leadVecIter = leadVecIter_;
	}
	// 点云配准
	void registration();
	// 获取变换矩阵
	Eigen::Matrix4f gettransMatrix()
	{
		return transMatrix;
	}
};
