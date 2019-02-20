#ifndef POISSON_SURFACE_TRIMMER_HPP
#define POISSON_SURFACE_TRIMMER_HPP
#include "surface_trimmer.h"
template<class Real, class Vertex>
void SmoothValues(std::vector<Vertex>& vertices, const std::vector<std::vector<int> >& polygons)
{
	std::vector<int> count(vertices.size());
	std::vector<Real> sums(vertices.size(), 0);
	for (size_t i = 0; i < polygons.size(); ++i)
	{
		int sz = int(polygons[i].size());
		for (int j = 0; j < sz; ++j)
		{
			int j1 = j, j2 = (j + 1) % sz;
			int v1 = polygons[i][j1], v2 = polygons[i][j2];
			count[v1]++, count[v2]++;
			sums[v1] += vertices[v2].value, sums[v2] += vertices[v1].value;
		}
	}
	for (size_t i = 0; i < vertices.size(); ++i)
		vertices[i].value = (sums[i] + vertices[i].value) / (count[i] + 1);
}

template<class Real, class Vertex>
void SmoothPoints(std::vector<Vertex>& vertices, const std::vector<std::vector<int> >& polygons)
{
	std::vector<int> count(vertices.size());
	std::vector<std::vector<Real> > sums(vertices.size(),std::vector<Real>(3));
	for (size_t i = 0; i < polygons.size(); ++i)
	{
		int sz = int(polygons[i].size());
		for (int j = 0; j < sz; ++j)
		{
			int j1 = j, j2 = (j + 1) % sz;
			int v1 = polygons[i][j1], v2 = polygons[i][j2];
			count[v1]++, count[v2]++;

			sums[v1][0] += vertices[v2].point.coords[0];
			sums[v1][1] += vertices[v2].point.coords[1];
			sums[v1][2] += vertices[v2].point.coords[2];

			sums[v2][0] += vertices[v1].point.coords[0];
			sums[v2][1] += vertices[v1].point.coords[1];
			sums[v2][2] += vertices[v1].point.coords[2];
		}
	}
	for (size_t i = 0; i < vertices.size(); ++i)
	{
		vertices[i].point.coords[0] = (sums[i][0] + vertices[i].point.coords[0]) / (count[i] + 1);
		vertices[i].point.coords[1] = (sums[i][1] + vertices[i].point.coords[1]) / (count[i] + 1);
		vertices[i].point.coords[2] = (sums[i][2] + vertices[i].point.coords[2]) / (count[i] + 1);
	}
}

template<class Real, class Vertex>
void SplitPolygon(const std::vector<int> &polygon, std::vector<Vertex>& vertices, std::vector<std::vector<int> >* ltPolygons, std::vector<std::vector<int> >* gtPolygons,
	std::vector<bool>* ltFlags, std::vector<bool>* gtFlags, std::unordered_map<long long, int>& vertexTable, Real hole_scale, Real default_radius)
{
	hole_scale = hole_scale < 1.0 ? 1.0 : hole_scale;
	float hole_radius = hole_scale * default_radius;
	float square_radius = hole_radius * hole_radius;
	int sz = int(polygon.size());
	std::vector<bool> gt(sz);
	int gtCount = 0;
	for (int j = 0; j < sz; ++j)
	{
		gt[j] = (vertices[polygon[j]].value < square_radius);
		if (gt[j])
			gtCount++;
	}
	if (gtCount == sz)
	{
		if (gtPolygons)
			gtPolygons->push_back(polygon);
		if (gtFlags)
			gtFlags->push_back(false);
	}
	else if (gtCount == 0)
	{
		if (ltPolygons)
			ltPolygons->push_back(polygon);
		if (ltFlags)
			ltFlags->push_back(false);
	}
	else
	{
		int start;
		for (start = 0; start < sz; ++start)
		{
			if (gt[start] && !gt[(start + sz - 1) % sz])
				break;
		}
		bool gtFlag = true;
		std::vector<int> poly;

		// Add the initial vertex
		{
			int j1 = (start + int(sz) - 1) % sz, j2 = start;
			int v1 = polygon[j1], v2 = polygon[j2];
			int vIdx;
			std::unordered_map<long long, int>::iterator iter = vertexTable.find(EdgeKey(v1, v2));
			if (iter == vertexTable.end())
			{
				vertexTable[EdgeKey(v1, v2)] = vIdx = int(vertices.size());
				vertices.push_back(InterpolateVertices(vertices[v1], vertices[v2], square_radius));
			}
			else
				vIdx = iter->second;
			poly.push_back(vIdx);
		}

		for (int k = 0; k <= sz; ++k)
		{
			int j1 = (k + start + sz - 1) % sz, j2 = (k + start) % sz;
			int v1 = polygon[j1], v2 = polygon[j2];
			if (gt[j2] == gtFlag)
				poly.push_back(v2);
			else
			{
				int vIdx;
				std::unordered_map<long long, int>::iterator iter = vertexTable.find(EdgeKey(v1, v2));
				if (iter == vertexTable.end())
				{
					vertexTable[EdgeKey(v1, v2)] = vIdx = int(vertices.size());
					vertices.push_back(InterpolateVertices(vertices[v1], vertices[v2], square_radius));
				}
				else
					vIdx = iter->second;
				poly.push_back(vIdx);
				if (gtFlag)
				{
					if (gtPolygons)
						gtPolygons->push_back(poly);
					if (ltFlags)
						ltFlags->push_back(true);
				}
				else
				{
					if (ltPolygons)
						ltPolygons->push_back(poly);
					if (gtFlags)
						gtFlags->push_back(true);
				}
				poly.clear();
				poly.push_back(vIdx);
				poly.push_back(v2);
				gtFlag = !gtFlag;
			}
		}
	}
}

template<class Real, class Vertex>
void Triangulate(const std::vector<Vertex>& vertices, const std::vector<std::vector<int> >& polygons, std::vector<std::vector<int> >& triangles)
{
	triangles.clear();
	for (size_t i = 0; i < polygons.size(); ++i)
	{
		if (polygons.size() > 3)
		{
			MinimalAreaTriangulation<Real> mat;
			std::vector<Point3D<Real> > _vertices(polygons[i].size());
			std::vector<TriangleIndex> _triangles;
			for (int j = 0; j < int(polygons[i].size()); ++j)
				_vertices[j] = vertices[polygons[i][j]].point;
			mat.GetTriangulation(_vertices, _triangles);

			// Add the triangles to the mesh
			size_t idx = triangles.size();
			triangles.resize(idx + _triangles.size());
			for (int j = 0; j < int(_triangles.size()); ++j)
			{
				triangles[idx + j].resize(3);
				for (int k = 0; k < 3; ++k)
					triangles[idx + j][k] = polygons[i][_triangles[j].idx[k]];
			}
		}
		else if (polygons[i].size() == 3)
			triangles.push_back(polygons[i]);
	}
}

template<class Vertex>
void RemoveHangingVertices(std::vector<Vertex>& vertices, std::vector<std::vector<int> >& polygons)
{
	std::unordered_map<int, int> vMap;
	std::vector<bool> vertexFlags(vertices.size(), false);
	for (size_t i = 0; i < polygons.size(); ++i)
		for (size_t j = 0; j < polygons[i].size(); ++j)
			vertexFlags[polygons[i][j]] = true;
	int vCount = 0;
	for (int i = 0; i < int(vertices.size()); ++i)
		if (vertexFlags[i])
			vMap[i] = vCount++;
	for (size_t i = 0; i < polygons.size(); ++i)
		for (size_t j = 0; j < polygons[i].size(); ++j)
			polygons[i][j] = vMap[polygons[i][j]];

	std::vector<Vertex> _vertices(vCount);
	for (int i = 0; i < int(vertices.size()); ++i)
		if (vertexFlags[i])
			_vertices[vMap[i]] = vertices[i];

	vertices = _vertices;
}
void SetConnectedComponents(const std::vector<std::vector<int> >& polygons, std::vector<std::vector<int> >& components)
{
	std::vector<int> polygonRoots(polygons.size());
	for (size_t i = 0; i < polygons.size(); ++i)
		polygonRoots[i] = int(i);

	std::unordered_map<long long, int> edgeTable;
	for (size_t i = 0; i < polygons.size(); ++i)
	{
		int sz = int(polygons[i].size());
		for (int j = 0; j < sz; ++j)
		{
			int j1 = j, j2 = (j + 1) % sz;
			int v1 = polygons[i][j1];
			int v2 = polygons[i][j2];
			long long eKey = EdgeKey(v1, v2);
			std::unordered_map<long long, int>::iterator iter = edgeTable.find(eKey);
			if (iter == edgeTable.end())
				edgeTable[eKey] = int(i);
			else
			{
				int p = iter->second;
				while (polygonRoots[p] != p)
				{
					int temp = polygonRoots[p];
					polygonRoots[p] = int(i);
					p = temp;
				}
				polygonRoots[p] = int(i);
			}
		}
	}
	for (size_t i = 0; i < polygonRoots.size(); ++i)
	{
		int p = int(i);
		while (polygonRoots[p] != p)
			p = polygonRoots[p];
		int root = p;
		p = int(i);
		while (polygonRoots[p] != p)
		{
			int temp = polygonRoots[p];
			polygonRoots[p] = root;
			p = temp;
		}
	}
	int cCount = 0;
	std::unordered_map<int, int> vMap;
	for (int i = 0; i < (int)polygonRoots.size(); ++i)
		if (polygonRoots[i] == i)
			vMap[i] = cCount++;
	components.resize(cCount);
	for (int i = 0; i < int(polygonRoots.size()); ++i)
		components[vMap[polygonRoots[i]]].push_back(i);
}
template<class Real, class Vertex>
void TrimSurface(std::vector<Vertex>& vertices, std::vector<std::vector<int> >& polygons, int smooth_iter, float trim, float aRatio)
{
	for (int i = 0; i < smooth_iter; ++i)
		SmoothValues<Real, Vertex >(vertices, polygons);

	float min, max;
	min = max = vertices[0].value;
	for (size_t i = 0; i < vertices.size(); ++i)
	{
		min = std::min<float>(min, vertices[i].value);
		max = std::max<float>(max, vertices[i].value);
	}
	printf("Value Range: [%f,%f]\n", min, max);
	hash_map<long long, int > vertexTable;
	std::vector<std::vector<int> > ltPolygons, gtPolygons;
	std::vector<bool> ltFlags, gtFlags;

	for (size_t i = 0; i < polygons.size(); ++i)
		SplitPolygon(polygons[i], vertices, &ltPolygons, &gtPolygons, &ltFlags, &gtFlags, vertexTable, trim);
	if (aRatio > 0)
	{
		std::vector<std::vector<int> > _ltPolygons, _gtPolygons;
		std::vector<std::vector<int> > ltComponents, gtComponents;
		SetConnectedComponents(ltPolygons, ltComponents);
		SetConnectedComponents(gtPolygons, gtPolygons);
		std::vector<double> ltAreas(ltComponents.size(), 0.), gtAreas(gtComponents.size(), 0.);
		std::vector<bool> ltComponentFlags(ltComponents.size(), false), gtComponentFlags(gtComponents.size(), false);
		double area = 0.;
		for (size_t i = 0; i < ltComponents.size(); ++i)
		{
			for (size_t j = 0; j < ltComponents[i].size(); ++j)
			{
				ltAreas[i] += PolygonArea<float, Vertex >(vertices, ltPolygons[ltComponents[i][j]]);
				ltComponentFlags[i] = (ltComponentFlags[i] || ltFlags[ltComponents[i][j]]);
			}
			area += ltAreas[i];
		}

		for (size_t i = 0; i < gtComponents.size(); ++i)
		{
			for (size_t j = 0; j < gtComponents[i].size(); ++j)
			{
				gtAreas[i] += PolygonArea<float, Vertex >(vertices, gtPolygons[gtComponents[i][j]]);
				gtComponentFlags[i] = (gtComponentFlags[i] || gtFlags[gtComponents[i][j]]);
			}
			area += gtAreas[i];
		}
		for (size_t i = 0; i < ltComponents.size(); ++i)
		{
			if (ltAreas[i] < area * aRatio && ltComponentFlags[i])
				for (size_t j = 0; j < ltComponents[i].size(); ++j)
					_gtPolygons.push_back(ltPolygons[ltComponents[i][j]]);
			else
				for (size_t j = 0; j < ltComponents[i].size(); ++j)
					_ltPolygons.push_back(ltPolygons[ltComponents[i][j]]);
		}
		for (size_t i = 0; i < gtComponents.size(); ++i)
		{
			if (gtAreas[i] < area * aRatio && gtComponentFlags[i])
				for (size_t j = 0; j < gtComponents[i].size(); ++j)
					_ltPolygons.push_back(gtPolygons[gtComponents[i][j]]);
			else
				for (size_t j = 0; j < gtComponents[i].size(); ++j)
					_gtPolygons.push_back(gtPolygons[gtComponents[i][j]]);
		}
		ltPolygons = _ltPolygons;
		gtPolygons = _gtPolygons;
	}

	RemoveHangingVertices<Vertex >(vertices, gtPolygons);
	polygons.clear();
	polygons = gtPolygons;
}
#endif