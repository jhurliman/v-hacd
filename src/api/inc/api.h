#pragma once

#include <stdint.h>

#pragma pack(push, 8)
struct VHACDSession {
	// Maximum number of convex hulls to produce (default = 64, range = 1 - 1024)
	int32_t maxConvexHulls = 64;
	// Maximum number of voxels generated during the voxelization stage
	// (default = 100,000, range = 10,000 - 64,000,000)
	int32_t resolution = 100000;
	// Maximum allowed concavity (default = 0.0025, range = 0.0 - 1.0)
	double concavity = 0.0025;
	// Controls the granularity of the search for the "best" clipping plane
	// (default = 4, range = 1 - 16)
	int32_t planeDownsampling = 4;
	// Controls the precision of the convex - hull generation process during
	// the clipping plane selection stage (default = 4, range = 1 - 16)
	int32_t convexhullDownsampling = 4;
	// Controls the bias toward clipping along symmetry planes
	// (default = 0.05, range = 0.0 - 1.0)
	double alpha = 0.05;
	// Controls the bias toward clipping along revolution axes
	// (default = 0.05, range = 0.0 - 1.0)
	double beta = 0.05;
	// Enable / disable normalizing the mesh before applying the convex
	// decomposition (default = 0, range = { 0,1 })
	int32_t pca = 0;
	// 0: voxel - based approximate convex decomposition, 1 : tetrahedron - based
	// approximate convex decomposition(default = 0, range = { 0,1 })
	int32_t mode = 0;
	// Controls the maximum number of triangles per convex hull
	// (default = 64, range = 4 - 1024)
	int32_t maxNumVerticesPerCH = 64;
	// Controls the adaptive sampling of the generated convex hulls
	// (default = 0.0001, range = 0.0 - 0.01)
	double minVolumePerCH = 0.0001;
	// Enable / disable approximation when computing convex hulls
	// (default = true)
	int32_t convexHullApproximation = 1;
	// Project the output convex hull vertices onto the original source mesh to
	// increase the floating point accuracy of the results (default = true)
	int32_t projectHullVertices = 1;
	// Enable / disable OpenCL acceleration (default = false)
	int32_t oclAcceleration = 0;
	// OpenCL platform id (default = 0, range = 0 - # OCL platforms)
	int32_t oclPlatformID = 0;
	// OpenCL device id (default = 0, range = 0 - # OCL devices)
	int32_t oclDeviceID = 0;

	double* outPoints;
	int32_t* outTriangles;
	int32_t* outPointOffsets;
	int32_t* outTriangleOffsets;
	int32_t outNumPoints;
	int32_t outNumTriangles;
	int32_t outNumMeshes;
};
#pragma pack(pop)

extern "C" __declspec(dllexport) bool VHACDGetOpenCLPlatforms(char outPlatforms[8][64]);

extern "C" __declspec(dllexport) bool VHACDGetOpenCLDevices(uint32_t platformIndex, char outDevices[8][64]);

extern "C" __declspec(dllexport) bool VHACDConvexDecomposition(
	const double& inPoints, int32_t inNumPoints, const int32_t& inTriangles,
	int32_t inNumTriangles, VHACDSession& session);

extern "C" __declspec(dllexport) void VHACDShutdown(VHACDSession& session);
