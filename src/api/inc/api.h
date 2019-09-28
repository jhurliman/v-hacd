#pragma once

#include <stdint.h>

extern "C" __declspec(dllexport) bool VHACDConvexDecomposition(
	const double* inPoints, uint32_t inNumPoints, const int32_t* inTriangles, uint32_t inNumTriangles,
	double** outPoints, uint32_t* outNumPoints, int32_t** outTriangles, uint32_t* outNumTriangles,
	int32_t** outPointOffsets, int32_t** outTriangleOffsets, uint32_t* outNumMeshes);

extern "C" __declspec(dllexport) void VHACDDestroy(double** points, int32_t** triangles, int32_t** pointOffsets, int32_t** triangleOffsets);
