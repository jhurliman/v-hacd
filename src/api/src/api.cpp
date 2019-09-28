#include "api.h"
#include "VHACD.h"

#include <algorithm>
#include <vector>

extern "C" bool VHACDConvexDecomposition(
	const double* inPoints, uint32_t inNumPoints, const int32_t* inTriangles, uint32_t inNumTriangles,
	double** outPoints, uint32_t* outNumPoints, int32_t** outTriangles, uint32_t* outNumTriangles,
	int32_t** outPointOffsets, int32_t** outTriangleOffsets, uint32_t* outNumMeshes) {

	VHACD::IVHACD::Parameters params;
	/*params.m_alpha;
	params.m_beta;
	params.m_callback;
	params.m_concavity;
	params.m_convexhullApproximation;
	params.m_convexhullDownsampling;
	params.m_maxConvexHulls;
	params.m_maxNumVerticesPerCH;
	params.m_minVolumePerCH;
	params.m_mode;*/
	params.m_oclAcceleration = false;
	/*params.m_pca;
	params.m_planeDownsampling;
	params.m_projectHullVertices;
	params.m_resolution;*/

	VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();

	//bool res = interfaceVHACD->OCLInit(oclHelper.GetDevice(), &myLogger);

	// Compute convex hulls
	bool res = interfaceVHACD->Compute(inPoints, inNumPoints, reinterpret_cast<const uint32_t*>(inTriangles), inNumTriangles, params);

	if (!res) {
		*outPoints = nullptr;
		*outTriangles = nullptr;
		*outPointOffsets = nullptr;
		*outTriangleOffsets = nullptr;
		*outNumPoints = 0;
		*outNumTriangles = 0;
		*outNumMeshes = 0;
		return false;
	}

	uint32_t numPoints = 0;
	uint32_t numTriangles = 0;
	uint32_t numConvexHulls = interfaceVHACD->GetNConvexHulls();
	std::vector<VHACD::IVHACD::ConvexHull> convexHulls;
	convexHulls.resize(numConvexHulls);

	// Retrieve all of the computed convex hulls and count total points/triangles
	for (uint32_t i = 0; i < numConvexHulls; ++i) {
		VHACD::IVHACD::ConvexHull& convexHull = convexHulls[i];
		interfaceVHACD->GetConvexHull(i, convexHull);
		numPoints += convexHull.m_nPoints;
		numTriangles += convexHull.m_nTriangles;
	}

	// Put all the geometry data into contiguous output arrays
	*outPoints = new double[numPoints * 3];
	*outTriangles = new int32_t[numTriangles * 3];
	*outPointOffsets = new int32_t[numConvexHulls];
	*outTriangleOffsets = new int32_t[numConvexHulls];
	*outNumPoints = numPoints;
	*outNumTriangles = numTriangles;
	*outNumMeshes = numConvexHulls;
	int32_t pointOffset = 0;
	int32_t triangleOffset = 0;
	for (uint32_t i = 0; i < numConvexHulls; ++i) {
		VHACD::IVHACD::ConvexHull& convexHull = convexHulls[i];

		std::copy(convexHull.m_points, convexHull.m_points + convexHull.m_nPoints * 3, (*outPoints) + pointOffset);
		std::copy(convexHull.m_triangles, convexHull.m_triangles + convexHull.m_nTriangles * 3, (*outTriangles) + triangleOffset);

		(*outPointOffsets)[i] = pointOffset;
		(*outTriangleOffsets)[i] = triangleOffset;

		pointOffset += convexHull.m_nPoints * 3;
		triangleOffset += convexHull.m_nTriangles * 3;
	}

	return true;
}

extern "C" void VHACDDestroy(double** points, int32_t** triangles, int32_t** pointOffsets, int32_t** triangleOffsets) {
	delete (*points);
	delete (*triangles);
	delete (*pointOffsets);
	delete (*triangleOffsets);

	*points = nullptr;
	*triangles = nullptr;
	*pointOffsets = nullptr;
	*triangleOffsets = nullptr;
}
