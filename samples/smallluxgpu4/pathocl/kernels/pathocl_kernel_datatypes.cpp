#include <string>
namespace slg { namespace ocl {
std::string KernelSource_PathOCL_DataTypes = 
"#line 1 \"pathocl_kernel_datatypes.cl\"\n"
"\n"
"/***************************************************************************\n"
" *   Copyright (C) 1998-2010 by authors (see AUTHORS.txt )                 *\n"
" *                                                                         *\n"
" *   This file is part of LuxRays.                                         *\n"
" *                                                                         *\n"
" *   LuxRays is free software; you can redistribute it and/or modify       *\n"
" *   it under the terms of the GNU General Public License as published by  *\n"
" *   the Free Software Foundation; either version 3 of the License, or     *\n"
" *   (at your option) any later version.                                   *\n"
" *                                                                         *\n"
" *   LuxRays is distributed in the hope that it will be useful,            *\n"
" *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *\n"
" *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *\n"
" *   GNU General Public License for more details.                          *\n"
" *                                                                         *\n"
" *   You should have received a copy of the GNU General Public License     *\n"
" *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *\n"
" *                                                                         *\n"
" *   LuxRays website: http://www.luxrender.net                             *\n"
" ***************************************************************************/\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// Some OpenCL specific definition\n"
"//------------------------------------------------------------------------------\n"
"\n"
"#if defined(SLG_OPENCL_KERNEL)\n"
"\n"
"#if defined(PARAM_USE_PIXEL_ATOMICS)\n"
"#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable\n"
"#endif\n"
"\n"
"#if defined(PARAM_HAS_SUNLIGHT) & !defined(PARAM_DIRECT_LIGHT_SAMPLING)\n"
"Error: PARAM_HAS_SUNLIGHT requires PARAM_DIRECT_LIGHT_SAMPLING !\n"
"#endif\n"
"\n"
"#ifndef TRUE\n"
"#define TRUE 1\n"
"#endif\n"
"\n"
"#ifndef FALSE\n"
"#define FALSE 0\n"
"#endif\n"
"\n"
"#endif\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// Frame buffer data types\n"
"//------------------------------------------------------------------------------\n"
"\n"
"typedef struct {\n"
"	Spectrum c;\n"
"	float count;\n"
"} Pixel;\n"
"\n"
"typedef struct {\n"
"	float alpha;\n"
"} AlphaPixel;\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// Sample data types\n"
"//------------------------------------------------------------------------------\n"
"\n"
"typedef struct {\n"
"	Spectrum radiance;\n"
"\n"
"	unsigned int pixelIndex;\n"
"} RandomSample;\n"
"\n"
"typedef struct {\n"
"	Spectrum radiance;\n"
"\n"
"	float totalI;\n"
"\n"
"	// Using ushort here totally freeze the ATI driver\n"
"	unsigned int largeMutationCount, smallMutationCount;\n"
"	unsigned int current, proposed, consecutiveRejects;\n"
"\n"
"	float weight;\n"
"	Spectrum currentRadiance;\n"
"	float currentAlpha;\n"
"} MetropolisSampleWithAlphaChannel;\n"
"\n"
"typedef struct {\n"
"	Spectrum radiance;\n"
"\n"
"	float totalI;\n"
"\n"
"	// Using ushort here totally freeze the ATI driver\n"
"	unsigned int largeMutationCount, smallMutationCount;\n"
"	unsigned int current, proposed, consecutiveRejects;\n"
"\n"
"	float weight;\n"
"	Spectrum currentRadiance;\n"
"} MetropolisSampleWithoutAlphaChannel;\n"
"\n"
"#if defined(SLG_OPENCL_KERNEL)\n"
"\n"
"#if (PARAM_SAMPLER_TYPE == 0)\n"
"typedef RandomSample Sample;\n"
"#endif\n"
"\n"
"#if (PARAM_SAMPLER_TYPE == 1)\n"
"#if defined(PARAM_ENABLE_ALPHA_CHANNEL)\n"
"typedef MetropolisSampleWithAlphaChannel Sample;\n"
"#else\n"
"typedef MetropolisSampleWithoutAlphaChannel Sample;\n"
"#endif\n"
"#endif\n"
"\n"
"#endif\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// Indices of Sample related u[] array\n"
"//------------------------------------------------------------------------------\n"
"\n"
"#if defined(SLG_OPENCL_KERNEL)\n"
"\n"
"#define IDX_SCREEN_X 0\n"
"#define IDX_SCREEN_Y 1\n"
"#if defined(PARAM_CAMERA_HAS_DOF)\n"
"#define IDX_DOF_X 2\n"
"#define IDX_DOF_Y 3\n"
"#define IDX_BSDF_OFFSET 4\n"
"#else\n"
"#define IDX_BSDF_OFFSET 2\n"
"#endif\n"
"\n"
"// Relative to IDX_BSDF_OFFSET + PathDepth * SAMPLE_SIZE\n"
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) && defined(PARAM_HAS_PASSTHROUGHT)\n"
"\n"
"#define IDX_PASSTROUGHT 0\n"
"#define IDX_BSDF_X 1\n"
"#define IDX_BSDF_Y 2\n"
"#define IDX_DIRECTLIGHT_X 3\n"
"#define IDX_DIRECTLIGHT_Y 4\n"
"#define IDX_DIRECTLIGHT_Z 5\n"
"#define IDX_DIRECTLIGHT_W 6\n"
"#define IDX_DIRECTLIGHT_A 7\n"
"#define IDX_RR 8\n"
"\n"
"#define SAMPLE_SIZE 9\n"
"\n"
"#elif defined(PARAM_DIRECT_LIGHT_SAMPLING)\n"
"\n"
"#define IDX_BSDF_X 0\n"
"#define IDX_BSDF_Y 1\n"
"#define IDX_DIRECTLIGHT_X 2\n"
"#define IDX_DIRECTLIGHT_Y 3\n"
"#define IDX_DIRECTLIGHT_Z 4\n"
"#define IDX_DIRECTLIGHT_W 5\n"
"#define IDX_DIRECTLIGHT_A 6\n"
"#define IDX_RR 7\n"
"\n"
"#define SAMPLE_SIZE 8\n"
"\n"
"#elif defined(PARAM_HAS_PASSTHROUGHT)\n"
"\n"
"#define IDX_PASSTROUGHT 0\n"
"#define IDX_BSDF_X 1\n"
"#define IDX_BSDF_Y 2\n"
"#define IDX_RR 3\n"
"\n"
"#define SAMPLE_SIZE 4\n"
"\n"
"#else\n"
"\n"
"#define IDX_BSDF_X 0\n"
"#define IDX_BSDF_Y 1\n"
"#define IDX_RR 2\n"
"\n"
"#define SAMPLE_SIZE 3\n"
"\n"
"#endif\n"
"\n"
"#if (PARAM_SAMPLER_TYPE == 0)\n"
"#define TOTAL_U_SIZE 2\n"
"#endif\n"
"\n"
"#if (PARAM_SAMPLER_TYPE == 1)\n"
"#define TOTAL_U_SIZE (IDX_BSDF_OFFSET + PARAM_MAX_PATH_DEPTH * SAMPLE_SIZE)\n"
"#endif\n"
"\n"
"#endif\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// GPUTask data types\n"
"//------------------------------------------------------------------------------\n"
"\n"
"typedef enum {\n"
"	RT_NEXT_VERTEX,\n"
"	GENERATE_DL_RAY,\n"
"	RT_DL_RAY,\n"
"	GENERATE_NEXT_VERTEX_RAY,\n"
"	SPLAT_SAMPLE\n"
"} PathState;\n"
"\n"
"typedef struct {\n"
"	PathState state;\n"
"	unsigned int depth;\n"
"\n"
"	Spectrum throughput;\n"
"	BSDF bsdf;\n"
"} PathStateBase;\n"
"\n"
"typedef struct {\n"
"	float bouncePdf;\n"
"	int specularBounce;\n"
"\n"
"	Ray nextPathRay;\n"
"	Spectrum nextThroughput;\n"
"	// Radiance to add to the result if light source is visible\n"
"	Spectrum lightRadiance;\n"
"} PathStateDirectLight;\n"
"\n"
"typedef struct {\n"
"	unsigned int vertexCount;\n"
"	float alpha;\n"
"} PathStateAlphaChannel;\n"
"\n"
"// This is defined only under OpenCL\n"
"#if defined(SLG_OPENCL_KERNEL)\n"
"\n"
"typedef struct {\n"
"	// The task seed\n"
"	Seed seed;\n"
"\n"
"	// The set of Samples assigned to this task\n"
"	Sample sample;\n"
"\n"
"	// The state used to keep track of the rendered path\n"
"	PathStateBase pathStateBase;\n"
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING)\n"
"	PathStateDirectLight directLightState;\n"
"#endif\n"
"#if defined(PARAM_ENABLE_ALPHA_CHANNEL)\n"
"	PathStateAlphaChannel alphaChannelState;\n"
"#endif\n"
"} GPUTask;\n"
"\n"
"#endif\n"
"\n"
"typedef struct {\n"
"	unsigned int sampleCount;\n"
"} GPUTaskStats;\n"
"\n"
"//------------------------------------------------------------------------------\n"
"\n"
"#define MAT_MATTE 0\n"
"#define MAT_AREALIGHT 1\n"
"#define MAT_MIRROR 2\n"
"#define MAT_GLASS 3\n"
"#define MAT_MATTEMIRROR 4\n"
"#define MAT_METAL 5\n"
"#define MAT_MATTEMETAL 6\n"
"#define MAT_ALLOY 7\n"
"#define MAT_ARCHGLASS 8\n"
"#define MAT_NULL 9\n"
"\n"
"typedef struct {\n"
"    float r, g, b;\n"
"} MatteParam;\n"
"\n"
"typedef struct {\n"
"    float gain_r, gain_g, gain_b;\n"
"} AreaLightParam;\n"
"\n"
"typedef struct {\n"
"    float r, g, b;\n"
"    int specularBounce;\n"
"} MirrorParam;\n"
"\n"
"typedef struct {\n"
"    float refl_r, refl_g, refl_b;\n"
"    float refrct_r, refrct_g, refrct_b;\n"
"    float ousideIor, ior;\n"
"    float R0;\n"
"    int reflectionSpecularBounce, transmitionSpecularBounce;\n"
"} GlassParam;\n"
"\n"
"typedef struct {\n"
"	MatteParam matte;\n"
"	MirrorParam mirror;\n"
"	float matteFilter, totFilter, mattePdf, mirrorPdf;\n"
"} MatteMirrorParam;\n"
"\n"
"typedef struct {\n"
"    float r, g, b;\n"
"    float exponent;\n"
"    int specularBounce;\n"
"} MetalParam;\n"
"\n"
"typedef struct {\n"
"	MatteParam matte;\n"
"	MetalParam metal;\n"
"	float matteFilter, totFilter, mattePdf, metalPdf;\n"
"} MatteMetalParam;\n"
"\n"
"typedef struct {\n"
"    float diff_r, diff_g, diff_b;\n"
"	float refl_r, refl_g, refl_b;\n"
"    float exponent;\n"
"    float R0;\n"
"    int specularBounce;\n"
"} AlloyParam;\n"
"\n"
"typedef struct {\n"
"    float refl_r, refl_g, refl_b;\n"
"    float refrct_r, refrct_g, refrct_b;\n"
"	float transFilter, totFilter, reflPdf, transPdf;\n"
"	bool reflectionSpecularBounce, transmitionSpecularBounce;\n"
"} ArchGlassParam;\n"
"\n"
"typedef struct {\n"
"	unsigned int type;\n"
"	union {\n"
"		MatteParam matte;\n"
"        AreaLightParam areaLight;\n"
"		MirrorParam mirror;\n"
"        GlassParam glass;\n"
"		MatteMirrorParam matteMirror;\n"
"        MetalParam metal;\n"
"        MatteMetalParam matteMetal;\n"
"        AlloyParam alloy;\n"
"        ArchGlassParam archGlass;\n"
"	} param;\n"
"} Material;\n"
"\n"
"//------------------------------------------------------------------------------\n"
"\n"
"typedef struct {\n"
"	Point v0, v1, v2;\n"
"	Normal normal;\n"
"	float area;\n"
"	float gain_r, gain_g, gain_b;\n"
"} TriangleLight;\n"
"\n"
"typedef struct {\n"
"	float shiftU, shiftV;\n"
"	Spectrum gain;\n"
"	unsigned int width, height;\n"
"} InfiniteLight;\n"
"\n"
"typedef struct {\n"
"	Vector sundir;\n"
"	Spectrum gain;\n"
"	float turbidity;\n"
"	float relSize;\n"
"	// XY Vectors for cone sampling\n"
"	Vector x, y;\n"
"	float cosThetaMax;\n"
"	Spectrum suncolor;\n"
"} SunLight;\n"
"\n"
"typedef struct {\n"
"	Spectrum gain;\n"
"	float thetaS;\n"
"	float phiS;\n"
"	float zenith_Y, zenith_x, zenith_y;\n"
"	float perez_Y[6], perez_x[6], perez_y[6];\n"
"} SkyLight;\n"
"\n"
"//------------------------------------------------------------------------------\n"
"\n"
"typedef struct {\n"
"	unsigned int rgbPage, rgbPageOffset;\n"
"	unsigned int alphaPage, alphaPageOffset;\n"
"	unsigned int width, height;\n"
"} TexMap;\n"
"\n"
"typedef struct {\n"
"	float uScale, vScale, uDelta, vDelta;\n"
"} TexMapInfo;\n"
"\n"
"typedef struct {\n"
"	float uScale, vScale, uDelta, vDelta;\n"
"	float scale;\n"
"} BumpMapInfo;\n"
"\n"
"typedef struct {\n"
"	float uScale, vScale, uDelta, vDelta;\n"
"} NormalMapInfo;\n"
; } }
