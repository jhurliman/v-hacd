#include "api.h"
#include "VHACD.h"
#include "oclHelper.h"

#include <algorithm>
#include <vector>

extern "C" bool VHACDGetOpenCLPlatforms(char outPlatforms[8][64]) {
	// Clear all existing platform names
	memset(outPlatforms, 0, 8 * 64);

	// Query for the number of platforms
	cl_uint numPlatforms;
	cl_int lastError = clGetPlatformIDs(1, nullptr, &numPlatforms);
	if (lastError != CL_SUCCESS)
		return false;

	// Query for platformIDs
	cl_platform_id* platforms = new cl_platform_id[numPlatforms];
	lastError = clGetPlatformIDs(numPlatforms, platforms, nullptr);
	if (lastError != CL_SUCCESS) {
		delete[] platforms;
		return false;
	}

	// Iterate over each platformID and query the platform vendor name
	for (cl_uint i = 0; i < numPlatforms && i < 8; ++i) {
		lastError = clGetPlatformInfo(platforms[i], CL_PLATFORM_VERSION, 64, outPlatforms[i], nullptr);
		if (lastError != CL_SUCCESS) {
			delete[] platforms;
			return false;
		}
	}

	delete[] platforms;
	return true;
}

extern "C" bool VHACDGetOpenCLDevices(uint32_t platformIndex, char outDevices[8][64]) {
	cl_uint numPlatforms;
	cl_int lastError = clGetPlatformIDs(1, nullptr, &numPlatforms);
	if (lastError != CL_SUCCESS || platformIndex >= numPlatforms) {
		return false;
	}

	cl_platform_id* platforms = new cl_platform_id[numPlatforms];
	lastError = clGetPlatformIDs(numPlatforms, platforms, nullptr);
	if (lastError != CL_SUCCESS) {
		delete[] platforms;
		return false;
	}

	cl_platform_id platform = platforms[platformIndex];

	// Clear all existing device names
	memset(outDevices, 0, 8 * 64);

	// Query for the number of devices
	cl_uint numDevices;
	lastError = clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, 0, nullptr, &numDevices);
	if (lastError != CL_SUCCESS) {
		return false;
	}

	// Query for deviceIDs
	cl_device_id* devices = new cl_device_id[numDevices];
	lastError = clGetDeviceIDs(platform, CL_DEVICE_TYPE_ALL, numDevices, devices, nullptr);
	if (lastError != CL_SUCCESS) {
		delete[] devices;
		return false;
	}

	// Iterate over each deviceID and query the device name
	for (cl_uint i = 0; i < numDevices && i < 8; ++i) {
		lastError = clGetDeviceInfo(devices[i], CL_DEVICE_NAME, 64, outDevices[i], nullptr);
		if (lastError != CL_SUCCESS) {
			delete[] devices;
			return false;
		}
	}

	delete[] devices;
	return true;
}

extern "C" bool VHACDConvexDecomposition(
	const double& inPoints, int32_t inNumPoints, const int32_t& inTriangles,
	int32_t inNumTriangles, VHACDSession& session) {

	VHACD::IVHACD::Parameters params;
	params.m_alpha = session.alpha;
	params.m_beta = session.beta;
	//params.m_callback;
	params.m_concavity = session.concavity;
	params.m_convexhullApproximation = uint32_t(session.convexHullApproximation);
	params.m_convexhullDownsampling = uint32_t(session.convexhullDownsampling);
	params.m_maxConvexHulls = uint32_t(session.maxConvexHulls);
	params.m_maxNumVerticesPerCH = uint32_t(session.maxNumVerticesPerCH);
	params.m_minVolumePerCH = session.minVolumePerCH;
	params.m_mode = uint32_t(session.mode);
	params.m_oclAcceleration = uint32_t(session.oclAcceleration);
	params.m_pca = uint32_t(session.pca);
	params.m_planeDownsampling = uint32_t(session.planeDownsampling);
	params.m_projectHullVertices = session.projectHullVertices;
	params.m_resolution = session.resolution;

	VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();

	OCLHelper oclHelper;
	if (session.oclAcceleration)
	{
		oclHelper.InitPlatform(session.oclPlatformID);
		oclHelper.InitDevice(session.oclDeviceID);
		if (!interfaceVHACD->OCLInit(oclHelper.GetDevice()))
		{
			params.m_oclAcceleration = 0;
		}
	}

	// Compute convex hulls
	const uint32_t& inUTriangles = reinterpret_cast<const uint32_t&>(inTriangles);
	if (!interfaceVHACD->Compute(&inPoints, uint32_t(inNumPoints), &inUTriangles, uint32_t(inNumTriangles), params)) {
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
	session.outPoints = new double[numPoints * 3];
	session.outTriangles = new int32_t[numTriangles * 3];
	session.outPointOffsets = new int32_t[numConvexHulls];
	session.outTriangleOffsets = new int32_t[numConvexHulls];
	session.outNumPoints = numPoints;
	session.outNumTriangles = numTriangles;
	session.outNumMeshes = numConvexHulls;
	int32_t pointOffset = 0;
	int32_t triangleOffset = 0;
	for (uint32_t i = 0; i < numConvexHulls; ++i) {
		VHACD::IVHACD::ConvexHull& convexHull = convexHulls[i];

		std::copy(convexHull.m_points, convexHull.m_points + convexHull.m_nPoints * 3, session.outPoints + pointOffset);
		std::copy(convexHull.m_triangles, convexHull.m_triangles + convexHull.m_nTriangles * 3, session.outTriangles + triangleOffset);

		session.outPointOffsets[i] = pointOffset;
		session.outTriangleOffsets[i] = triangleOffset;

		pointOffset += convexHull.m_nPoints * 3;
		triangleOffset += convexHull.m_nTriangles * 3;
	}

	return true;
}

extern "C" void VHACDShutdown(VHACDSession& session) {
	delete[] session.outPoints;
	delete[] session.outTriangles;
	delete[] session.outPointOffsets;
	delete[] session.outTriangleOffsets;

	session.outPoints = nullptr;
	session.outTriangles = nullptr;
	session.outPointOffsets = nullptr;
	session.outTriangleOffsets = nullptr;
}
