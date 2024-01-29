/**
 *  \file   TINConstruct.h
 *  \brief
 *  \author wxy
 *  \email  wuxiaoyang22@mails.ucas.ac.cn
 *  \date   2023/12/21
 *  \note   
 */
//
#pragma once
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <vector>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "descriptor.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                       Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                    Delaunay;
typedef Kernel::Point_2                                                Point;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;

class TINConstruct
{
 public:
  //cloudPtr cloud;
  void construct(cloudPtr seedCloud,std::vector<Line>& line_all);
  void construct_grid(cloudPtr cloud_lowest,cloudPtr input_cloud,
	  cloudPtr seed_cloud,std::vector<float>& height);
  //double calculateArea(double x0, double y0, double x1, double y1, double x2, double y2);
  float interpolate(pcl::PointXYZ p1,pcl::PointXYZ p2,pcl::PointXYZ p3,pcl::PointXYZ p0);




};
