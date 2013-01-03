#include <string>
namespace luxrays { namespace ocl {
std::string KernelSource_texture_funcs = 
"#line 2 \"texture_funcs.cl\"\n"
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
"// ConstFloat texture\n"
"//------------------------------------------------------------------------------\n"
"\n"
"float3 ConstFloatTexture_GetColorValue(__global Texture *texture, const float2 uv) {\n"
"	return texture->constFloat.value;\n"
"}\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// ConstFloat3 texture\n"
"//------------------------------------------------------------------------------\n"
"\n"
"float3 ConstFloat3Texture_GetColorValue(__global Texture *texture, const float2 uv) {\n"
"	return vload3(0, &texture->constFloat3.color.r);\n"
"}\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// ConstFloat4 texture\n"
"//------------------------------------------------------------------------------\n"
"\n"
"float3 ConstFloat4Texture_GetColorValue(__global Texture *texture, const float2 uv) {\n"
"	return vload3(0, &texture->constFloat4.color.r);\n"
"}\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// ImageMap texture\n"
"//------------------------------------------------------------------------------\n"
"\n"
"#if defined(PARAM_HAS_IMAGEMAPS)\n"
"\n"
"__global float *ImageMap_GetPixelsAddress(\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_0)\n"
"	__global float *imageMapBuff0,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_1)\n"
"	__global float *imageMapBuff1,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_2)\n"
"	__global float *imageMapBuff2,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_3)\n"
"	__global float *imageMapBuff3,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_4)\n"
"	__global float *imageMapBuff4,\n"
"#endif\n"
"	const uint page, const uint offset\n"
"    ) {\n"
"    switch (page) {\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_1)\n"
"        case 1:\n"
"            return &imageMapBuff1[offset];\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_2)\n"
"        case 2:\n"
"            return &imageMapBuff2[offset];\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_3)\n"
"        case 3:\n"
"            return &imageMapBuff3[offset];\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_4)\n"
"        case 4:\n"
"            return &imageMapBuff4[offset];\n"
"#endif\n"
"        default:\n"
"        case 0:\n"
"            return &imageMapBuff0[offset];\n"
"    }\n"
"}\n"
"\n"
"float3 ImageMap_GetTexel_Color(__global float *pixels,\n"
"		const uint width, const uint height, const uint channelCount,\n"
"		const int s, const int t) {\n"
"	const uint u = Mod(s, width);\n"
"	const uint v = Mod(t, height);\n"
"\n"
"	const uint index = channelCount * (v * width + u);\n"
"\n"
"	return vload3(0, &pixels[index]);\n"
"}\n"
"\n"
"float3 ImageMap_GetColor(__global float *pixels,\n"
"		const uint width, const uint height, const uint channelCount,\n"
"		const float u, const float v) {\n"
"	const float s = u * width - 0.5f;\n"
"	const float t = v * height - 0.5f;\n"
"\n"
"	const int s0 = (int)floor(s);\n"
"	const int t0 = (int)floor(t);\n"
"\n"
"	const float ds = s - s0;\n"
"	const float dt = t - t0;\n"
"\n"
"	const float ids = 1.f - ds;\n"
"	const float idt = 1.f - dt;\n"
"\n"
"	const float3 c0 = ImageMap_GetTexel_Color(pixels, width, height, channelCount, s0, t0);\n"
"	const float3 c1 = ImageMap_GetTexel_Color(pixels, width, height, channelCount, s0, t0 + 1);\n"
"	const float3 c2 = ImageMap_GetTexel_Color(pixels, width, height, channelCount, s0 + 1, t0);\n"
"	const float3 c3 = ImageMap_GetTexel_Color(pixels, width, height, channelCount, s0 + 1, t0 + 1);\n"
"\n"
"	const float k0 = ids * idt;\n"
"	const float k1 = ids * dt;\n"
"	const float k2 = ds * idt;\n"
"	const float k3 = ds * dt;\n"
"\n"
"	return (k0 * c0 + k1 *c1 + k2 * c2 + k3 * c3);\n"
"}\n"
"\n"
"float3 ImageMapInstance_GetColor(\n"
"	__global ImageMapInstanceParam *imageMapInstance,\n"
"	__global ImageMap *imageMapDescs,\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_0)\n"
"	__global float *imageMapBuff0,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_1)\n"
"	__global float *imageMapBuff1,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_2)\n"
"	__global float *imageMapBuff2,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_3)\n"
"	__global float *imageMapBuff3,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_4)\n"
"	__global float *imageMapBuff4,\n"
"#endif\n"
"	const float2 uv) {\n"
"	__global ImageMap *imageMap = &imageMapDescs[imageMapInstance->imageMapIndex];\n"
"	__global float *pixels = ImageMap_GetPixelsAddress(\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_0)\n"
"		imageMapBuff0,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_1)\n"
"		imageMapBuff1,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_2)\n"
"		imageMapBuff2,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_3)\n"
"		imageMapBuff3,\n"
"#endif\n"
"#if defined(PARAM_IMAGEMAPS_PAGE_4)\n"
"		imageMapBuff4,\n"
"#endif\n"
"		imageMap->pageIndex, imageMap->pixelsIndex\n"
"		);\n"
"\n"
"	const float2 scale = vload2(0, &imageMapInstance->uScale);\n"
"	const float2 delta = vload2(0, &imageMapInstance->uDelta);\n"
"	const float2 mapUV = uv * scale + delta;\n"
"\n"
"	return imageMapInstance->gain * ImageMap_GetColor(\n"
"			pixels,\n"
"			imageMap->width, imageMap->height, imageMap->channelCount,\n"
"			mapUV.s0, mapUV.s1);\n"
"}\n"
"\n"
"#endif\n"
"\n"
"//------------------------------------------------------------------------------\n"
"\n"
"float3 Texture_GetColorValue(__global Texture *texture, const float2 uv) {\n"
"	switch (texture->type) {\n"
"		case CONST_FLOAT:\n"
"			return ConstFloatTexture_GetColorValue(texture, uv);\n"
"		case CONST_FLOAT3:\n"
"			return ConstFloat3Texture_GetColorValue(texture, uv);\n"
"		case CONST_FLOAT4:\n"
"			return ConstFloat4Texture_GetColorValue(texture, uv);\n"
"		default:\n"
"			return BLACK;\n"
"	}\n"
"}\n"
; } }
