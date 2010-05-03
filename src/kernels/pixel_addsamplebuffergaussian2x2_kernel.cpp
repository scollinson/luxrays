#include "luxrays/kernels/kernels.h"
std::string luxrays::KernelSource_Pixel_AddSampleBufferGaussian2x2 = 
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
"// NOTE: this kernel assume samples do not overlap\n"
"\n"
"#define Gaussian2x2_xWidth 2.f\n"
"#define Gaussian2x2_yWidth 2.f\n"
"#define Gaussian2x2_invXWidth (1.f / 2.f)\n"
"#define Gaussian2x2_invYWidth (1.f / 2.f)\n"
"\n"
"typedef struct {\n"
"	float r, g, b;\n"
"} Spectrum;\n"
"\n"
"typedef struct {\n"
"	Spectrum radiance;\n"
"	float weight;\n"
"} SamplePixel;\n"
"\n"
"typedef struct {\n"
"	float screenX, screenY;\n"
"	Spectrum radiance;\n"
"} SampleBufferElem;\n"
"\n"
"static int Ceil2Int(const float val) {\n"
"	return (int)ceil(val);\n"
"}\n"
"\n"
"static int Floor2Int(const float val) {\n"
"	return (int)floor(val);\n"
"}\n"
"\n"
"static void AddSample(__global SamplePixel *sp, const float4 sample) {\n"
"    float4 weight = (float4)(sample.w, sample.w, sample.w, 1.f);\n"
"    __global float4 *p = (__global float4 *)sp;\n"
"    *p += weight * sample;\n"
"}\n"
"\n"
"__kernel __attribute__((reqd_work_group_size(64, 1, 1))) void PixelAddSampleBufferGaussian2x2(\n"
"	const unsigned int width,\n"
"	const unsigned int height,\n"
"	__global SamplePixel *sampleFrameBuffer,\n"
"	const unsigned int sampleCount,\n"
"	__global SampleBufferElem *sampleBuff,\n"
"	const unsigned int FilterTableSize,\n"
"    __global float *Gaussian2x2_filterTable,\n"
"	__local int *ifxBuff, __local int *ifyBuff) {\n"
"	const unsigned int index = get_global_id(0);\n"
"	if (index >= sampleCount)\n"
"		return;\n"
"\n"
"	__global SampleBufferElem *sampleElem = &sampleBuff[index];\n"
"    const float4 sample = (float4)(sampleElem->radiance.r, sampleElem->radiance.g, sampleElem->radiance.b, 1.f);\n"
"\n"
"	const float dImageX = sampleElem->screenX - 0.5f;\n"
"	const float dImageY = sampleElem->screenY - 0.5f;\n"
"	const int x0 = Ceil2Int(dImageX - Gaussian2x2_xWidth);\n"
"	const int x1 = Floor2Int(dImageX + Gaussian2x2_xWidth);\n"
"	const int y0 = Ceil2Int(dImageY - Gaussian2x2_yWidth);\n"
"	const int y1 = Floor2Int(dImageY + Gaussian2x2_yWidth);\n"
"	if (x1 < x0 || y1 < y0 || x1 < 0 || y1 < 0)\n"
"		return;\n"
"\n"
"	// Loop over filter support and add sample to pixel arrays\n"
"	__local int *ifx = &(ifxBuff[16 * get_local_id(0)]);\n"
"	for (int x = x0; x <= x1; ++x) {\n"
"		const float fx = fabs((x - dImageX) *\n"
"				Gaussian2x2_invXWidth * FilterTableSize);\n"
"		ifx[x - x0] = min(Floor2Int(fx), (int)FilterTableSize - 1);\n"
"	}\n"
"\n"
"	__local int *ify = &(ifyBuff[16 * get_local_id(0)]);\n"
"	for (int y = y0; y <= y1; ++y) {\n"
"		const float fy = fabs((y - dImageY) *\n"
"				Gaussian2x2_invYWidth * FilterTableSize);\n"
"		ify[y - y0] = min(Floor2Int(fy), (int)FilterTableSize - 1);\n"
"	}\n"
"\n"
"	float filterNorm = 0.f;\n"
"	for (int y = y0; y <= y1; ++y) {\n"
"		for (int x = x0; x <= x1; ++x) {\n"
"			const int offset = ify[y - y0] * FilterTableSize + ifx[x - x0];\n"
"			filterNorm += Gaussian2x2_filterTable[offset];\n"
"		}\n"
"	}\n"
"	filterNorm = 1.f / filterNorm;\n"
"\n"
"	const int fx0 = max(x0, 0);\n"
"	const int fx1 = min(x1, (int)width - 1);\n"
"	const int fy0 = max(y0, 0);\n"
"	const int fy1 = min(y1, (int)height - 1);\n"
"\n"
"    for (int y = fy0; y <= fy1; ++y) {\n"
"        const unsigned int offset = y * width;\n"
"\n"
"		for (int x = fx0; x <= fx1; ++x) {\n"
"            const int tabOffset = ify[y - y0] * FilterTableSize + ifx[x - x0];\n"
"			sample.w = Gaussian2x2_filterTable[tabOffset] * filterNorm;\n"
"\n"
"            AddSample(&sampleFrameBuffer[offset + x], sample);\n"
"        }\n"
"	}\n"
"}\n"
;
