// spdlog
#include <spdlog/spdlog.h>
// local
#include "SC2.h"

bool myCompareVec(std::pair<float, int> &a, std::pair<float, int> &b)
{
	return a.first > b.first;
}

bool compareCor(std::pair<int, int> &a, std::pair<int, int> &b)
{
	return a.first > b.first;
}

double calDis(const pcl::PointXYZ &p, const pcl::PointXYZ &q)
{
	return sqrt(pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2));

}

// 计算主特征向量
template <class T>
void calLeadEigenVec(const T &Mat, std::vector<float> &vec, const int iterNum)
{
	// power iteration 算法计算主特征向量
	int n = Mat.cols();
	vec.resize(n);
	// Eigen::MatrixXf V = Eigen::MatrixXf::Random(n, 1).array().abs();
	Eigen::MatrixXf V = Eigen::MatrixXf::Ones(n, 1);
	for (int i = 0; i < iterNum; ++i)
	{
		V = Mat * V;
		double sum = 0;
		for (int j = 0; j < n; ++j)
		{
			sum += pow(V(j, 0), 2);
		}
		for (int j = 0; j < n; ++j)
		{
			V(j, 0) /= sum;
		}
	}

	for (int i = 0; i < n; ++i)
	{
		vec[i] = V(i, 0);
	}
}

// 计算种子corres经过矩阵变换后能变换正确的个数
int calCount(const pcl::PointCloud<pcl::PointXYZ>::Ptr sour, const pcl::PointCloud<pcl::PointXYZ>::Ptr tar,
	const std::vector<int>& seeds, Eigen::Matrix4f& trans, const std::vector<std::pair<int, int>>& corres, float thre)
{
	int count = 0;
	for (int i = 0; i < seeds.size(); ++i)
	{
		pcl::PointXYZ ts = sour->points[corres[seeds[i]].first];
		pcl::PointXYZ tt = tar->points[corres[seeds[i]].second];
		float dx = trans(0, 0) * ts.x + trans(0, 1) * ts.y + trans(0, 2) * ts.z + trans(0, 3) - tt.x;
		float dy = trans(1, 0) * ts.x + trans(1, 1) * ts.y + trans(1, 2) * ts.z + trans(1, 3) - tt.y;
		float dz = trans(2, 0) * ts.x + trans(2, 1) * ts.y + trans(2, 2) * ts.z + trans(2, 3) - tt.z;
		if (sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2)) < thre)
		{
			++count;
		}
	}

	return count;
}

SC2::SC2()
{
	dthr = 0.1;                //SC矩阵阈值
	pointThre = 0.3;           //点正确阈值
	k1 = 30;                   //第一阶段corres个数
	k2 = 20;                   //第二阶段corres个数
	leadVecIter = 20;          //power iteration算法迭代次数
	seedsRatio = 0.2;          //种子点比例
	radius = 0.2;              //非极大线性抑制范围
}

SC2::~SC2()
{
}

void SC2::setcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr sour_, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_)
{
	sour = sour_;
	tar = tar_;
}

void SC2::registration()
{
	if (sour == nullptr || tar == nullptr || sour->size() == 0 || tar->size() == 0 || corres.empty())
	{
		printf("Data Error!!\n");
		exit(1);
	}
	calScMat();
	pickSeeds();
	calScHardMat();

	calSc2Mat();

	transMatrix = calBestTrans();
}

// 选择种子点
void SC2::pickSeeds()
{
	int sn = seedsRatio * scMat.cols();
	seeds.reserve(sn);
	std::vector<float> leadVec;
	calLeadEigenVec<Eigen::MatrixXf>(scMat, leadVec, leadVecIter);
	// 从特征向量中选择最大的sn个点，且一定范围内不重复选点（非极大线性抑制）
	std::vector<std::pair<float, int>> sortLeadVec(leadVec.size());
	for (int i = 0; i < leadVec.size(); ++i)
	{
		sortLeadVec[i].first = leadVec[i];
		sortLeadVec[i].second = i;
	}
	sort(sortLeadVec.begin(), sortLeadVec.end(), myCompareVec); //ֱ�ӱȽϴ�С

	// 非极大线性抑制
	for (int i = 80; i < sortLeadVec.size(); ++i) //!!
	{
		if (sortLeadVec[i].first == 0)
			continue;
		pcl::PointXYZ &a = sour->points[corres[sortLeadVec[i].second].first];
		for (int j = i + 1; j < sortLeadVec.size(); ++j)
		{
			pcl::PointXYZ &b = sour->points[corres[sortLeadVec[j].second].first];
			double dd = sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
			if (dd <= radius)
			{
				sortLeadVec[j].first = 0;
			}
		}
	}

	// 经过非极大线性抑制后再次排序，并取前sn个为种子点
	sort(sortLeadVec.begin(), sortLeadVec.end(), myCompareVec);
	for (int i = 0; i < sn; ++i)
	{
		// seeds[i] = sortLeadVec[i].second;
		if (sortLeadVec[i].first != 0)
		{
			seeds.push_back(sortLeadVec[i].second);
		}
	}

	printf("pick seed pair success! The number of seeds is %d\n", (int)seeds.size());
}

// 计算一阶空间兼容矩阵（SC mat）
void SC2::calScMat()
{
	int n = corres.size();
	scMat.resize(n, n);

	for (int i = 0; i < n; ++i)
	{
		scMat(i, i) = 0;
		for (int j = i + 1; j < n; ++j)
		{
			double dis1 = calDis(sour->points[corres[i].first], sour->points[corres[j].first]);
			double dis2 = calDis(tar->points[corres[i].second], tar->points[corres[j].second]);
			double dis = abs(dis1 - dis2);
			double score = 1 - pow(dis, 2) / pow(dthr, 2);
			scMat(i, j) = score < 0 ? 0 : score;
			scMat(j, i) = scMat(i, j);
		}
	}
	printf("calculate first order sc matrix success!\n");
}

// 计算一阶空间兼容矩阵的二值化矩阵，为计算二阶空间兼容矩阵做准备
void SC2::calScHardMat()
{
	Eigen::MatrixXd &hardMat = seedHardMat;
	int m = scMat.cols(), sn = seeds.size();
	hardMat.resize(sn, sn);
	for (int i = 0; i < sn; ++i)
	{
		for (int j = i; j < sn; ++j)
		{
			hardMat(i, j) = scMat(seeds[i], seeds[j]) <= 0 ? 0 : 1;
			hardMat(j, i) = hardMat(i, j);
		}
	}
	printf("calculate seed hard sc matrix success!\n");
}

// 计算二阶空间兼容矩阵
void SC2::calSc2Mat()
{
	Eigen::MatrixXd &hardMat = seedHardMat;
	sc2Mat = hardMat * hardMat;
	printf("calculate the second order SC matrix success!\n");
}

void SC2::calWeight(std::vector<int>& Col)
{
	int n = Col.size();
	weights.resize(n, n);

	Eigen::MatrixXf k2SC(n, n);
	for (int i = 0; i < n; ++i)
	{
		for (int j = i; j < n; ++j)
		{
			k2SC(i, j) = scMat(Col[i], Col[j]);
			k2SC(j, i) = k2SC(i, j);
		}
	}
	Eigen::MatrixXf SC_ = k2SC * k2SC;
	Eigen::MatrixXf SC_2(n, n);
	for (int i = 0; i < n; ++i)
	{
		for (int j = 0; j < n; ++j)
		{
			SC_2(i, j) = SC_(i, j) * k2SC(i, j);
		}
	}
	std::vector<float> vec;
	calLeadEigenVec(SC_2, vec, 40);
	for (int i = 0; i < vec.size(); ++i)
	{
		weights(i, i) = vec[i];
	}
}

// 计算得到最终的矩阵变换
Eigen::Matrix4f SC2::calBestTrans()
{
	int n = seeds.size();
	int maxCount = -1, index = -1;
	Eigen::Matrix4f finalTrans;

	for (int i = 0; i < n; ++i)
	{
		Eigen::Matrix4f curTrans = calTrans(i);
		// cout<<curTrans<<endl;
		int count = calCount(sour, tar, seeds, curTrans, corres, pointThre);
		if (count >= maxCount)
		{
			maxCount = count;
			finalTrans = curTrans;
			index = i;
		}
	}
	spdlog::info("max count: {} index: {} pointThre: {}\n ", maxCount, index, pointThre);
	return finalTrans;
}

// 计算每个共识集合的变换矩阵
Eigen::Matrix4f SC2::calTrans(const int index)
{
	int n = sc2Mat.cols();
	int fn = k1; // first
	int sn = k2;
	std::vector<int> firstColIndex(fn);             // 存储第一阶段共识集合index
	std::vector<int> secondColIndex(sn);            // 存储第二阶段共识集合index
	std::vector<std::pair<int, int>> firstCorr(n);  // 存储待排序的值： 相似度，位置
	std::vector<std::pair<int, int>> secondCorr(fn);
	Eigen::MatrixXf localScMat(fn, fn);
	Eigen::MatrixXd localHardMat(fn, fn);
	Eigen::MatrixXd localSc2Mat(fn, fn);
	Eigen::MatrixXf leadVec;
	Eigen::MatrixXf cloudP(3, sn), cloudQ(3, sn);
	Eigen::MatrixXf cloudPtem(3, sn), cloudQtem(3, sn);
	Eigen::MatrixXf transTem(3, sn);
	Eigen::MatrixXf H(3, 3);
	Eigen::Matrix4f trans;  // trans*P=Q
	Eigen::Matrix3f transR; // transR*p+transT=Q
	Eigen::MatrixXf transT = Eigen::MatrixXf::Zero(3, 1);
	Eigen::MatrixXf meanP = Eigen::MatrixXf::Zero(3, 1);
	Eigen::MatrixXf meanQ = Eigen::MatrixXf::Zero(3, 1);

	// 第一阶段，排序，取前k1个数作为共识集合
	for (int i = 0; i < n; ++i)
	{
		firstCorr[i].first = sc2Mat(index, i);
		firstCorr[i].second = i;
	}
	sort(firstCorr.begin(), firstCorr.end(), compareCor);
	firstColIndex[0] = seeds[index];
	for (int i = 1; i < fn; ++i)
	{
		firstColIndex[i] = seeds[firstCorr[i - 1].second];
	}

	// 计算localScMat、localHardMat、localSc2Mat
	for (int i = 0; i < fn; ++i)
	{
		for (int j = i + 1; j < fn; ++j)
		{
			localScMat(i, j) = scMat(firstColIndex[i], firstColIndex[j]);
			localScMat(j, i) = localScMat(i, j);
			localHardMat(i, j) = localScMat(i, j) <= 0 ? 0 : 1;
			localHardMat(j, i) = localHardMat(i, j);
		}
	}
	for (int i = 0; i < fn; ++i)
	{
		for (int j = i + 1; j < fn; ++j)
		{
			localSc2Mat(i, j) = 0;
			for (int l = 0; l < fn; ++l)
			{
				localSc2Mat(i, j) += localHardMat(i, l) * localHardMat(l, j);
				localSc2Mat(j, i) = localSc2Mat(i, j);
			}
		}
	}

	// 第二阶段，取前k2个
	for (int i = 0; i < fn; ++i)
	{
		secondCorr[i].first = localSc2Mat(0, i);
		secondCorr[i].second = i;
	}
	sort(secondCorr.begin(), secondCorr.end(), compareCor);
	secondColIndex[0] = index;
	for (int i = 1; i < sn; ++i)
	{
		secondColIndex[i] = firstColIndex[secondCorr[i - 1].second];
	}

	// 求权重
	// calWeight(secondColIndex);
	// svd分解计算变换矩阵
	for (int i = 0; i < sn; ++i)
	{
		pcl::PointXYZ &ts = sour->points[corres[secondColIndex[i]].first];
		pcl::PointXYZ &tt = tar->points[corres[secondColIndex[i]].second];
		cloudP(0, i) = ts.x;
		cloudP(1, i) = ts.y;
		cloudP(2, i) = ts.z;
		cloudQ(0, i) = tt.x;
		cloudQ(1, i) = tt.y;
		cloudQ(2, i) = tt.z;
		meanP += cloudP.col(i);
		meanQ += cloudQ.col(i);
	}
	meanP /= sn;
	meanQ /= sn;
	cloudPtem = cloudP;
	cloudQtem = cloudQ;
	for (int i = 0; i < sn; ++i)
	{
		cloudP.col(i) -= meanP;
		cloudQ.col(i) -= meanQ;
	}
	H = cloudP * cloudQ.transpose();
	Eigen::JacobiSVD<Eigen::MatrixXf> svdH(H, Eigen::DecompositionOptions::ComputeFullU | Eigen::DecompositionOptions::ComputeFullV);
	transR = svdH.matrixV() * svdH.matrixU().transpose();
	transTem = cloudQtem - transR * cloudPtem;

	for (int i = 0; i < sn; ++i)
	{
		transT(0, 0) += transTem(0, i);
		transT(1, 0) += transTem(1, i);
		transT(2, 0) += transTem(2, i);
	}
	transT /= sn;
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			trans(i, j) = transR(i, j);
		}
		trans(3, i) = 0;
		trans(i, 3) = transT(i, 0);
	}
	trans(3, 3) = 1;

	return trans;
}