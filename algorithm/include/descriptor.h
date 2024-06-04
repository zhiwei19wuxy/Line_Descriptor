/**
 *  \file   descriptor.h
 *  \brief
 *  \author wxy
 *  \email  wuxiaoyang22@mails.ucas.ac.cn
 *  \date   2023/12/21
 *  \note   
 */
//

#pragma once
#include<iostream>
#include<vector>
// pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr;
typedef pcl::PointCloud<pcl::PointXYZ> cloud;

struct Descriptor
{
  float angle;//两点连线与水平面的夹角
  float length;//两点连线的长度
  float height_low;//树高，更矮的那个
  float height_tall;//树高，更高的那个
};

struct Line
{
  int idx1;//起点，第一个在seed_cloud点云中的索引
  int idx2;//终点，第二个在seed_cloud点云中的索引
  Descriptor des;//边的描述子
};

void getAngle_Length(Line& line,cloudPtr seed_cloud);
void getDescriptor(std::vector<Line>& line_all,cloudPtr seed_cloud,cloudPtr input_cloud,float grid_size);

