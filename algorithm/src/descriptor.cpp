#include "descriptor.h"
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include "TINConstruct.h"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

void getAngle_Length(Line& line, cloudPtr seed_cloud)
{
	Eigen::Vector3f vector_z(0.0, 0.0, 1.0);
	pcl::PointXYZ point1(seed_cloud->points[line.idx1]);
	pcl::PointXYZ point2(seed_cloud->points[line.idx2]);
	//获取两点之间的向量,2-1,是从1指到2
	Eigen::Vector3f vector(point2.x - point1.x, point2.y - point1.y, point2.z - point1.z);
	// 检查Z坐标，如果point1的Z较大，反转向量方向
	if (seed_cloud->points[line.idx2].z < seed_cloud->points[line.idx1].z)
	{
		vector *= -1.0;
	}
	float angle = pcl::getAngle3D(vector, vector_z, true);
	line.des.angle = 90 - angle;

	// 计算两个点之间的欧几里得距离
	double distance = pcl::euclideanDistance(point1, point2);
	line.des.length = distance;
}

void getDescriptor(std::vector<Line>& line_all, cloudPtr seed_cloud, cloudPtr input_cloud, float grid_size)
{
	//划分格网，根据格网最低点构建TIN，将point点落在三角面片上获取地面z值，差值即为树高
	//格网最低点：为避免格网最低点为非地面点造成的误差，将格网最低点与其邻域格网比较，差值大于阈值的剔除

	//划分格网
	pcl::PointXYZ point_min, point_max;
	pcl::getMinMax3D(*input_cloud, point_min, point_max);
	//获取最大值和最小值
	cloudPtr cloud_minmax(new cloud);
	cloud_minmax->push_back(point_min);
	cloud_minmax->push_back(point_max);
	//pcl::io::savePCDFile<pcl::PointXYZ>("D:\\data\\registration\\TLS_1_minmax.pcd", *cloud_minmax);
	//如果min是最小的，可能会出现，相减为零的情况，因此，将其减小一点
//	point_min.x -= 0.00001;
//	point_min.y -= 0.00001;
	int rows = std::ceil((point_max.x - point_min.x) / grid_size);//计算行列数
	int cols = std::ceil((point_max.y - point_min.y) / grid_size);
	std::vector<std::vector<int>> grid_low;//最低点的索引
	std::vector<std::vector<std::vector<int>>> grid;//三维索引，存储全部的点
	grid_low.resize(rows);//初始化空间
	for (auto& row_low : grid_low)
	{
		row_low.resize(cols, -1);//初始化为-1

	}
	grid.resize(rows);//初始化grid的行
	for (auto& row : grid)
	{
		row.resize(cols);//列

	}
	for (int i = 0; i < input_cloud->size(); i++)
	{
		int row = std::ceil((input_cloud->points[i].x - point_min.x) / grid_size) - 1;
		int col = std::ceil((input_cloud->points[i].y - point_min.y) / grid_size) - 1;
		if(row==-1)
		{
			row=0;
		}
		if(col==-1)
		{
			col=0;
		}
		grid[row][col].push_back(i);
		if (grid_low[row][col] == -1)//如果是-1的话，将每个索引初始值认为是第一个
		{
			grid_low[row][col] = i;
		}
		else if (input_cloud->points[i].z < input_cloud->points[grid_low[row][col]].z)
		{
			grid_low[row][col] = i;//如果某个点比第一个点的z值还低，则将索引存到grid_low里面
		}
	}

	cloudPtr cloud_z0(new cloud);//z为0的格网最低点，为了进行k近邻索引，于是将z值赋值为0
	std::vector<std::pair<int, int>> idx_ij;
	for (int row = 0; row < grid_low.size(); row++)
	{
		for (int col = 0; col < grid_low[row].size(); col++)
		{
			if (grid_low[row][col] == -1)
			{
				continue;
			}
			cloud_z0->push_back(pcl::PointXYZ(
				input_cloud->points[grid_low[row][col]].x, input_cloud->points[grid_low[row][col]].y, 0));
			idx_ij.push_back(std::pair<int, int>(row, col));
			//height_grid.push_back(input_cloud->points[grid_low[i][j]].z);
		}
	}
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(cloud_z0);
	float threshold = 1.0;
	std::vector<std::pair<int, int>> idx_removed;// 待移除的网格最低点索引
	std::vector<int> point_indices;
	std::vector<float> point_distances;
	for (int i = 0; i < cloud_z0->size(); i++)
	{
//		kdtree.radiusSearch(cloud_z0->points[i],2*grid_size, point_indices, point_distances);
		kdtree.nearestKSearch(cloud_z0->points[i], 15, point_indices, point_distances);

		for (int j = 0; j < point_indices.size(); j++)
		{
			float delta_height = input_cloud->points[grid_low[idx_ij[i].first][idx_ij[i].second]].z
				- input_cloud->points[grid_low[idx_ij[point_indices[j]].first][idx_ij[point_indices[j]].second]].z;
			if (delta_height > threshold)
			{
				idx_removed.push_back(idx_ij[i]);//将他存储起来，方便后续移除
				break;
			}
		}
	}

	for (const auto& idx_removed_item : idx_removed)
	{
		grid_low[idx_removed_item.first][idx_removed_item.second] = -1;//将他标志起来，方便后续移除
	}

	//格网最低点，加四个角点
	cloudPtr cloud_lowest(new cloud);
	for (int i = 0; i < grid_low.size(); i++)
	{
		for (int j = 0; j < grid_low[i].size(); j++)
		{
			if (grid_low[i][j] == -1)
			{
				continue;
			}
			cloud_lowest->push_back(input_cloud->points[grid_low[i][j]]);
		}
	}
//	pcl::io::savePCDFile<pcl::PointXYZ>("D:\\data\\registration\\TLS_1_grid_lowest.pcd", *cloud_lowest);

	pcl::PointXYZ p1,p2,p3,p4;//p1左上角点，p2左下角点，p3右上角点，p4右下角点
	p1.x=point_min.x;
	p1.y=point_max.y;
	p1.z=0;
	p2.x=point_min.x;
	p2.y=point_min.y;
	p2.z=0;
	p3.x=point_max.x;
	p3.y=point_max.y;
	p3.z=0;
	p4.x=point_max.x;
	p4.y=point_min.y;
	p4.z=0;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_lowest;
	kdtree_lowest.setInputCloud(cloud_lowest);
	kdtree_lowest.nearestKSearch(p1, 1, point_indices, point_distances);
	p1.z=cloud_lowest->points[point_indices[0]].z;
	kdtree_lowest.nearestKSearch(p2, 1, point_indices, point_distances);
	p2.z=cloud_lowest->points[point_indices[0]].z;
	kdtree_lowest.nearestKSearch(p3, 1, point_indices, point_distances);
	p3.z=cloud_lowest->points[point_indices[0]].z;
	kdtree_lowest.nearestKSearch(p4, 1, point_indices, point_distances);
	p4.z=cloud_lowest->points[point_indices[0]].z;

	cloud_lowest->push_back(p1);
	cloud_lowest->push_back(p2);
	cloud_lowest->push_back(p3);
	cloud_lowest->push_back(p4);


	for (int i = 0; i < line_all.size(); i++)
	{
		getAngle_Length(line_all[i], seed_cloud);
	}
	//获取全部高程
	std::vector<float> height;
	TINConstruct T;
	T.construct_grid(cloud_lowest, input_cloud, seed_cloud, height);
	for(auto& line:line_all)
	{
		if(height[line.idx1]>height[line.idx2])
		{
			line.des.height_tall=height[line.idx1];
			line.des.height_low=height[line.idx2];
			int temp;
			temp=line.idx1;
			line.idx1=line.idx2;
			line.idx2=temp;//完成后，idx1对应更矮的树，idx2对应更高的树

		}
		else
		{
			line.des.height_low=height[line.idx1];
			line.des.height_tall=height[line.idx2];
		}
	}


}