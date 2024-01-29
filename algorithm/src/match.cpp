// system
# include <random>
# include <unordered_set>
// pcl
#include <pcl/octree/octree_search.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
// spdlog
#include <spdlog/spdlog.h>
// local
#include "SC2.h"
#include "match.h"

void BuildKDTree(const std::vector<std::vector<float>>& feature, KDTree* tree)
{
	auto rows = feature.size();
	auto dim = feature[0].size();
	std::vector<float> dataset(rows * dim);
	flann::Matrix<float> dataset_mat(&dataset[0], rows, dim);
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < dim; j++)
			dataset[i * dim + j] = feature[i][j];
	KDTree temp_tree(dataset_mat, flann::KDTreeSingleIndexParams(15));
	temp_tree.buildIndex();
	*tree = temp_tree;
}

void SearchKDTree(KDTree* tree, const std::vector<float>& feature, std::vector<int>& indices, std::vector<float>& dists, int nn)
{
	auto feature_temp = feature;
	flann::Matrix<float> query_mat(&feature_temp[0], 1, feature.size());

	indices.resize(nn);
	dists.resize(nn);
	flann::Matrix<int> indices_mat(&indices[0], 1, nn);
	flann::Matrix<float> dists_mat(&dists[0], 1, nn);

	tree->knnSearch(query_mat, indices_mat, dists_mat, nn, flann::SearchParams(128));
}

bool MatchPair(const std::vector<std::vector<float>>& feature_src,
	const std::vector<std::vector<float>>& feature_tgt,
	std::vector<std::pair<int, int>>& corres,
	std::vector<float>& feature_rmse)
{
	if (feature_src.empty() || feature_tgt.empty())
	{
		//spdlog::error("feature is empty.");
		return false;
	}

	KDTree sourFeaTree(flann::KDTreeSingleIndexParams(15));
	KDTree tarFeaTree(flann::KDTreeSingleIndexParams(15));
	std::vector<int> indices;
	std::vector<float> dists;
	BuildKDTree(feature_src, &sourFeaTree);
	BuildKDTree(feature_tgt, &tarFeaTree);
	// 只有当都互为最近邻时 才添加为corres
	for (int i = 0; i < feature_src.size(); ++i)
	{
		SearchKDTree(&tarFeaTree, feature_src[i], indices, dists, 1);
		int j = indices[0];
		SearchKDTree(&sourFeaTree, feature_tgt[j], indices, dists, 1);
		if (indices[0] == i)
		{
			corres.push_back(std::pair<int, int>(i, j));
			feature_rmse.push_back(dists[0]);
		}
	}

	//spdlog::info("match pair success,the number of pairs is {}.", corres.size());

	return true;
}

bool RT_RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_tgt,
	const std::vector<std::pair<int, int>>& corres,
	int k, float dist_thresh, Eigen::Matrix4f& rt)
{
	int max_num = 0;
	// 随机选取三对点，但不可重复
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> distribution(0, corres.size() - 1);
	for (int itnum = 0; itnum < k; itnum++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr points_src(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr points_tgt(new pcl::PointCloud<pcl::PointXYZ>);
		std::unordered_set<int> chosenNumbers;
		while (chosenNumbers.size() < 3)
		{
			int randomNumber = distribution(gen);
			// 检查是否已经选择了这个数字
			if (chosenNumbers.find(randomNumber) == chosenNumbers.end())
			{
				chosenNumbers.insert(randomNumber);
				points_src->push_back(xyz_src->points[corres[randomNumber].first]);
				points_tgt->push_back(xyz_tgt->points[corres[randomNumber].second]);
			}
		}

		// SVD
		Eigen::Matrix4f rt_temp;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> SVD;
		SVD.estimateRigidTransformation(*points_src, *points_tgt, rt_temp);

		int num = 0; //符合阈值的点对数量
		pcl::PointCloud<pcl::PointXYZ>::Ptr trans_src(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*xyz_src, *trans_src, rt_temp);
		for (const auto& pair : corres)
		{
			if (pcl::euclideanDistance(trans_src->points[pair.first], xyz_tgt->points[pair.second]) < dist_thresh)
				num++;
		}

		// 更新最优RT
		if (num > max_num)
		{
			max_num = num;
			rt = rt_temp;
		}

		// 符合阈值的点对超过总数的90%
		if (num > int(0.9 * corres.size()))
			break;
	}

	// 最优变换矩阵，符合阈值的点对低于总数的20%
//	if (max_num <= int(0.2 * corres.size()))
//	{
//		spdlog::warn("Less than 20 per cent.");
//		return false;
//	}

	return true;
}


bool RT_RANSAC2(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_tgt,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_temp_src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_temp_tgt,
	const std::vector<std::pair<int, int>>& corres,
	int k, float dist_thresh, Eigen::Matrix4f& rt)
{
	int max_num = 0;
	// 随机选取三对点，但不可重复
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> distribution(0, corres.size() - 1);

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(xyz_temp_tgt);
	std::vector<int> point_indices;
	std::vector<float> point_distances;
	std::vector<std::pair<int, int>> corres_max;

	for (int itnum = 0; itnum < k; itnum++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr points_src(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr points_tgt(new pcl::PointCloud<pcl::PointXYZ>);
		std::unordered_set<int> chosenNumbers;
		while (chosenNumbers.size() < 3)
		{
			int randomNumber = distribution(gen);
			// 检查是否已经选择了这个数字
			if (chosenNumbers.find(randomNumber) == chosenNumbers.end())
			{
				chosenNumbers.insert(randomNumber);
				points_src->push_back(xyz_src->points[corres[randomNumber].first]);
				points_tgt->push_back(xyz_tgt->points[corres[randomNumber].second]);
			}
		}

		// SVD
		Eigen::Matrix4f rt_temp;
		pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> SVD;
		SVD.estimateRigidTransformation(*points_src, *points_tgt, rt_temp);

		int num = 0; //符合阈值的点对数量
		pcl::PointCloud<pcl::PointXYZ>::Ptr trans_src(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*xyz_temp_src, *trans_src, rt_temp);

		std::vector<std::pair<int, int>> corres_LS;
		for(int i=0;i<trans_src->size();i++)
		{
			kdtree.nearestKSearch(trans_src->points[i], 1, point_indices, point_distances);
			if(sqrt(point_distances[0])<dist_thresh)
			{
				corres_LS.push_back(std::pair<int,int>(i,point_indices[0]));
			}
		}

		if(corres_LS.size()>corres_max.size())
		{
			corres_max.clear();
			corres_max=corres_LS;
		}
	}

//	Eigen::Matrix4f rt_temp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_src_LS(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr points_tgt_LS(new pcl::PointCloud<pcl::PointXYZ>);
	for(auto corres_iter:corres_max)
	{
		points_src_LS->push_back(xyz_temp_src->points[corres_iter.first]);
		points_tgt_LS->push_back(xyz_temp_tgt->points[corres_iter.second]);
	}

	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> SVD;
	SVD.estimateRigidTra nsformation(*points_src_LS, *points_tgt_LS, rt);

	// 最优变换矩阵，符合阈值的点对低于总数的20%
//	if (max_num <= int(0.2 * corres.size()))
//	{
//		spdlog::warn("Less than 20 per cent.");
//		return false;
//	}

	return true;
}

bool RT_SC2(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_src,
	pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_tgt,
	const std::vector<std::pair<int, int>>& corres,
	SC2_Params params, Eigen::Matrix4f& rt)
{
	SC2 sc2;
	sc2.setcloud(xyz_src, xyz_tgt);
	sc2.setcorres(corres);
	sc2.setdthr(params.dthr);
	sc2.setpointThre(params.pointThre);
	sc2.setseedsRatio(params.seedsRatio);
	sc2.setradius(params.radius);
	sc2.setk1(params.k1);
	sc2.setk2(params.k2);
	sc2.setleadVecIter(params.leadVecIter);
	sc2.registration();
	sc2.gettransMatrix();

	return true;
}
