#include <string>
namespace slg { namespace ocl {
std::string KernelSource_bsdf_types = 
"#line 2 \"bsdf_types.cl\"\n"
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
"typedef enum {\n"
"	NONE     = 0,\n"
"	DIFFUSE  = 1,\n"
"	GLOSSY   = 2,\n"
"	SPECULAR = 4,\n"
"	REFLECT  = 8,\n"
"	TRANSMIT = 16\n"
"} BSDFEventType;\n"
"\n"
"typedef int BSDFEvent;\n"
"\n"
"// This is defined only under OpenCL because of variable size structures\n"
"#if defined(SLG_OPENCL_KERNEL)\n"
"\n"
"typedef struct {\n"
"	HitPoint hitPoint;\n"
"\n"
"	unsigned int materialIndex;\n"
"#if (PARAM_DL_LIGHT_COUNT > 0)\n"
"	unsigned int triangleLightSourceIndex;\n"
"#endif\n"
"\n"
"	Frame frame;\n"
"} BSDF;\n"
"\n"
"#endif\n"
; } }