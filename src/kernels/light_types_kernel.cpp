#include <string>
namespace luxrays { namespace ocl {
std::string KernelSource_light_types = 
"#line 2 \"light_types.cl\"\n"
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
"	TYPE_IL, TYPE_IL_SKY, TYPE_SUN, TYPE_TRIANGLE\n"
"} LightSourceType;\n"
"\n"
"typedef struct {\n"
"	Spectrum gain;\n"
"	float shiftU, shiftV;\n"
"	ImageMapInstanceParam imageMapInstance;\n"
"} InfiniteLight;\n"
"\n"
"typedef struct {\n"
"	unsigned int materialIndex;\n"
"} TriangleLight;\n"
; } }
