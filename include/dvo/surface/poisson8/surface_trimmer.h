#ifndef POISSON_SURFACE_TRIMMER_H
#define POISSON_SURFACE_TRIMMER_H
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <algorithm>
#include "geometry.h"
#include "ply.h"
#include "mat.h"

long long EdgeKey(int key1, int key2)
{
	if (key1 < key2)
		return (((long long)key1) << 32) | ((long long)key2);
	else
		return (((long long)key2) << 32) | ((long long)key1);
}

template<class Real, class Vertex>
Vertex InterpolateVertices(const Vertex& v1, const Vertex& v2, Real value)
{
	typename Vertex::Wrapper _v1(v1), _v2(v2);
	if (_v1.value == _v2.value)
		return Vertex((_v1 + _v2) / Real(2.0));
	Real dx = (_v1.value - value) / (_v1.value - _v2.value);
	return Vertex(_v1*(1.f - dx) + _v2*dx);
}
template<class Real>
inline Point3D<Real> CrossProduct(Point3D<Real> p1, Point3D<Real> p2)
{
	return Point3D< Real >(p1[1] * p2[2] - p1[2] * p2[1], p1[2] * p2[0] - p1[0] * p2[2], p1[0] * p1[1] - p1[1] * p2[0]);
}
template<class Real>
double TriangleArea(Point3D<Real> v1, Point3D<Real> v2, Point3D<Real> v3)
{
	Point3D<Real> n = CrossProduct(v2 - v1, v3 - v1);
	return sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]) / 2.;
}
template<class Real, class Vertex>
double PolygonArea(const std::vector<Vertex>& vertices, const std::vector<int>& polygon)
{
	if (polygon.size() < 3)
		return 0.;
	else if (polygon.size() == 3)
		return TriangleArea(vertices[polygon[0]].point, vertices[polygon[1]].point, vertices[polygon[2]].point);
	else
	{
		Point3D<Real> center;
		for (size_t i = 0; i < polygon.size(); ++i)
			center += vertices[polygon[i]].point;
		center /= Real(polygon.size());
		double area = 0;
		for (size_t i = 0; i < polygon.size(); ++i)
			area += TriangleArea(center, vertices[polygon[i]].point, vertices[polygon[(i + 1) % polygon.size()]].point);
		return area;
	}
}
template<class Real, class Vertex>
void SmoothValues(std::vector<Vertex>& vertices, const std::vector<std::vector<int> >& polygons);

template<class Real, class Vertex>
void SmoothPoints(std::vector<Vertex>& vertices, const std::vector<std::vector<int> >& polygons);

template<class Real, class Vertex>
void SplitPolygon(const std::vector<int> &polygon, std::vector<Vertex>& vertices, std::vector<std::vector<int> >* ltPolygons, std::vector<std::vector<int> >* gtPolygons,
	std::vector<bool>* itFlags, std::vector<bool>* gtFlags, std::unordered_map<long long, int>& vertexTable, Real hole_scale, Real default_radius);

template<class Real, class Vertex>
void Triangulate(const std::vector<Vertex>& vertices, const std::vector<std::vector<int> >& polygons, std::vector<std::vector<int> >& triangles);

template<class Vertex>
void RemoveHangingVertices(std::vector<Vertex>& vertices, std::vector<std::vector<int> >& polygons);

void SetConnectedComponents(const std::vector<std::vector<int> >& polygons, std::vector<std::vector<int> >& components);

#include "surface_trimmer.hpp"

#endif