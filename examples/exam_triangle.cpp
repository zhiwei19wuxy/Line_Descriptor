#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel            Kernel;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned int, Kernel> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb>                       Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds>                    Delaunay;
typedef Kernel::Point_2                                                Point;

int main() {
  std::vector< std::pair<Point,unsigned> > points;
  points.push_back( std::make_pair( Point(1,1), 0 ) );
  points.push_back( std::make_pair( Point(1,2), 1 ) );
  points.push_back( std::make_pair( Point(1,3), 2 ) );
  points.push_back( std::make_pair( Point(2,1), 3 ) );
  points.push_back( std::make_pair( Point(2,2), 4 ) );
  points.push_back( std::make_pair( Point(2,3), 5 ) );

  Delaunay triangulation;
  triangulation.insert(points.begin(),points.end());

  for(Delaunay::Finite_edges_iterator fit = triangulation.finite_edges_begin();
	  fit != triangulation.finite_edges_end(); ++fit)
  {
	auto a=fit->first->vertex((fit->second + 1) % 3)->info();
	auto b=fit->first->vertex((fit->second + 2) % 3)->info();
	//auto c=fit->second->vertex(0)->info();
	std::cout<<"a: "<<a;
	std::cout<<" b: "<<b<<std::endl;
  }
}
