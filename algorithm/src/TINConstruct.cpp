#include"TINConstruct.h"
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>


void TINConstruct::construct(cloudPtr seedCloud,std::vector<Line>& line_all) {
  //读取点云数据
  std::vector<std::pair<Point, unsigned> > points;
  for (int i = 0; i < seedCloud->size(); i++) {
	points.push_back(std::make_pair(Point(seedCloud->points[i].x, seedCloud->points[i].y), i));
  }

  Delaunay triangulation;
  triangulation.insert(points.begin(), points.end());

  for (Delaunay::Finite_edges_iterator fit = triangulation.finite_edges_begin();
	   fit != triangulation.finite_edges_end(); ++fit) {
	Line line;
	line.idx1 = fit->first->vertex((fit->second + 1) % 3)->info();
	line.idx2 = fit->first->vertex((fit->second + 2) % 3)->info();
	line_all.push_back(line);
	//std::cout << "a: " << a << " b: " << b << std::endl;
  }

}

float TINConstruct::interpolate(pcl::PointXYZ p0,pcl::PointXYZ p1,pcl::PointXYZ p2,pcl::PointXYZ p)
{
	float areaABC=0.5*((p1.x-p0.x)*(p2.y-p0.y)-(p2.x-p0.x)*(p1.y-p0.y));
	float areaPBC=0.5*((p1.x-p.x)*(p2.y-p.y)-(p2.x-p.x)*(p1.y-p.y));
	float areaPCA=0.5*((p.x-p0.x)*(p2.y-p0.y)-(p2.x-p0.x)*(p.y-p0.y));

	double alpha = areaPBC / areaABC;
	double beta = areaPCA / areaABC;
	double gamma = 1 - alpha - beta;

	double interpolatedZ = alpha * p0.z + beta * p1.z + gamma * p2.z;
	return interpolatedZ;
}


void TINConstruct::construct_grid(cloudPtr cloud_lowest,cloudPtr input_cloud,
	cloudPtr seed_cloud,std::vector<float>& height)
{
	//读取点云数据
	std::vector<std::pair<Point, int>> points;
	for (int i=0;i<cloud_lowest->size();i++)
	{
		points.push_back(std::make_pair(Point(cloud_lowest->points[i].x,cloud_lowest->points[i].y),i));
	}

	Delaunay triangulation;
	triangulation.insert(points.begin(), points.end());

	//将seed_cloud落在三角网内部
	for(int i=0;i<seed_cloud->size();i++)
	{
		Delaunay::Face_handle loc;
		int li = 0;
		Delaunay::Locate_type lt;
		loc = triangulation.locate(Point(seed_cloud->points[i].x,seed_cloud->points[i].y), lt, li);

		pcl::PointXYZ p1,p2,p3;
		p1.x=CGAL::to_double(loc->vertex(0)->point().x());
		p1.y=CGAL::to_double(loc->vertex(0)->point().y());
		p1.z=cloud_lowest->points[loc->vertex(0)->info()].z;
		p2.x=CGAL::to_double(loc->vertex(1)->point().x());
		p2.y=CGAL::to_double(loc->vertex(1)->point().y());
		p2.z=cloud_lowest->points[loc->vertex(1)->info()].z;
		p3.x=CGAL::to_double(loc->vertex(2)->point().x());
		p3.y=CGAL::to_double(loc->vertex(2)->point().y());
		p3.z=cloud_lowest->points[loc->vertex(2)->info()].z;

		float height_triangle = interpolate(p1,p2,p3,seed_cloud->points[i]);
		float height_single=seed_cloud->points[i].z-height_triangle;
		height.push_back(height_single);
	}
}



void KDNetwork::buildNetwork(cloudPtr seedcloud, std::vector<Line>& line_all)
{
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(seedcloud);

	for (size_t i = 0; i < seedcloud->size(); ++i) {
		pcl::PointXYZ current_point = seedcloud->at(i);

		// 查询当前点的最近邻点
		std::vector<int> pointIdxNKNSearch;
		std::vector<float> pointNKNSquaredDistance;
		int K = 10; // 查询最近的 5 个邻域点
		if (kdtree.nearestKSearch(current_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
			// 遍历每个最近邻点
			for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j) {
				int neighbor_index = pointIdxNKNSearch[j];
				if (neighbor_index != i) {
					// 创建 Line 结构体并添加到 line_all 中
					Line line;
					line.idx1 = i;
					line.idx2 = neighbor_index;

					// 将当前 Line 添加到 line_all 中
					line_all.push_back(line);
				}
			}
		}
	}
}
