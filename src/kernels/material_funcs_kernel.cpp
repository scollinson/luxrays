#include <string>
namespace luxrays { namespace ocl {
std::string KernelSource_material_funcs = 
"#line 2 \"material_funcs.cl\"\n"
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
"// Matte material\n"
"//------------------------------------------------------------------------------\n"
"\n"
"float3 MatteMaterial_Sample(__global Material *material, __global Texture *texs,\n"
"		const float2 uv, const float3 fixedDir, float3 *sampledDir,\n"
"		const float u0, const float u1, \n"
"#if defined(PARAM_HAS_PASSTHROUGHT)\n"
"		const float passThroughEvent,\n"
"#endif\n"
"		float *pdfW, float *cosSampledDir, BSDFEvent *event) {\n"
"	if (fabs(fixedDir.z) < DEFAULT_COS_EPSILON_STATIC)\n"
"				return 0.f;\n"
"\n"
"	*sampledDir = (signbit(fixedDir.z) ? -1.f : 1.f) * CosineSampleHemisphereWithPdf(u0, u1, pdfW);\n"
"\n"
"	*cosSampledDir = fabs((*sampledDir).z);\n"
"	if (*cosSampledDir < DEFAULT_COS_EPSILON_STATIC)\n"
"		return 0.f;\n"
"\n"
"	*event = DIFFUSE | REFLECT;\n"
"\n"
"	const float3 kd = Texture_GetColorValue(&texs[material->matte.kdTexIndex], uv);\n"
"	return M_1_PI_F * kd;\n"
"}\n"
"\n"
"//------------------------------------------------------------------------------\n"
"// Mirror material\n"
"//------------------------------------------------------------------------------\n"
"\n"
"float3 MirrorMaterial_Sample(__global Material *material, __global Texture *texs,\n"
"		const float2 uv, const float3 fixedDir, float3 *sampledDir,\n"
"		const float u0, const float u1,\n"
"#if defined(PARAM_HAS_PASSTHROUGHT)\n"
"		const float passThroughEvent,\n"
"#endif\n"
"		float *pdfW, float *cosSampledDir, BSDFEvent *event) {\n"
"	*event = SPECULAR | REFLECT;\n"
"\n"
"	*sampledDir = (float3)(-fixedDir.x, -fixedDir.y, fixedDir.z);\n"
"	*pdfW = 1.f;\n"
"\n"
"	*cosSampledDir = fabs((*sampledDir).z);\n"
"	const float3 kr = Texture_GetColorValue(&texs[material->mirror.krTexIndex], uv);\n"
"	// The cosSampledDir is used to compensate the other one used inside the integrator\n"
"	return kr / (*cosSampledDir);\n"
"}\n"
"\n"
"//------------------------------------------------------------------------------\n"
"\n"
"float3 Material_Sample(__global Material *material, __global Texture *texs,\n"
"		const float2 uv, const float3 fixedDir, float3 *sampledDir,\n"
"		const float u0, const float u1,\n"
"#if defined(PARAM_HAS_PASSTHROUGHT)\n"
"		const float passThroughEvent,\n"
"#endif\n"
"		float *pdfW, float *cosSampledDir, BSDFEvent *event) {\n"
"	switch (material->type) {\n"
"		case MATTE:\n"
"			return MatteMaterial_Sample(material, texs, uv, fixedDir, sampledDir,\n"
"					u0, u1,\n"
"#if defined(PARAM_HAS_PASSTHROUGHT)\n"
"					passThroughEvent,\n"
"#endif\n"
"					pdfW, cosSampledDir, event);\n"
"		case MIRROR:\n"
"			return MirrorMaterial_Sample(material, texs, uv, fixedDir, sampledDir,\n"
"					u0, u1,\n"
"#if defined(PARAM_HAS_PASSTHROUGHT)\n"
"					passThroughEvent,\n"
"#endif\n"
"					pdfW, cosSampledDir, event);\n"
"		default:\n"
"			return 0.f;\n"
"	}\n"
"}\n"
; } }
