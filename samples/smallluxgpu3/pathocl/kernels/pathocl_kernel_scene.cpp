#include "pathocl/kernels/kernels.h" 
std::string luxrays::KernelSource_PathOCL_kernel_scene = 
"#line 2 \"patchocl_kernel_scene.cl\"\n" 
"/***************************************************************************\n" 
"*   Copyright (C) 1998-2010 by authors (see AUTHORS.txt )                 *\n" 
"*                                                                         *\n" 
"*   This file is part of LuxRays.                                         *\n" 
"*                                                                         *\n" 
"*   LuxRays is free software; you can redistribute it and/or modify       *\n" 
"*   it under the terms of the GNU General Public License as published by  *\n" 
"*   the Free Software Foundation; either version 3 of the License, or     *\n" 
"*   (at your option) any later version.                                   *\n" 
"*                                                                         *\n" 
"*   LuxRays is distributed in the hope that it will be useful,            *\n" 
"*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *\n" 
"*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *\n" 
"*   GNU General Public License for more details.                          *\n" 
"*                                                                         *\n" 
"*   You should have received a copy of the GNU General Public License     *\n" 
"*   along with this program.  If not, see <http://www.gnu.org/licenses/>. *\n" 
"*                                                                         *\n" 
"*   LuxRays website: http://www.luxrender.net                             *\n" 
"***************************************************************************/\n" 
"void TexMap_GetTexel(__global Spectrum *pixels, const uint width, const uint height,\n" 
"const int s, const int t, Spectrum *col) {\n" 
"const uint u = Mod(s, width);\n" 
"const uint v = Mod(t, height);\n" 
"const unsigned index = v * width + u;\n" 
"col->r = pixels[index].r;\n" 
"col->g = pixels[index].g;\n" 
"col->b = pixels[index].b;\n" 
"}\n" 
"float TexMap_GetAlphaTexel(__global float *alphas, const uint width, const uint height,\n" 
"const int s, const int t) {\n" 
"const uint u = Mod(s, width);\n" 
"const uint v = Mod(t, height);\n" 
"const unsigned index = v * width + u;\n" 
"return alphas[index];\n" 
"}\n" 
"void TexMap_GetColor(__global Spectrum *pixels, const uint width, const uint height,\n" 
"const float u, const float v, Spectrum *col) {\n" 
"const float s = u * width - 0.5f;\n" 
"const float t = v * height - 0.5f;\n" 
"const int s0 = (int)floor(s);\n" 
"const int t0 = (int)floor(t);\n" 
"const float ds = s - s0;\n" 
"const float dt = t - t0;\n" 
"const float ids = 1.f - ds;\n" 
"const float idt = 1.f - dt;\n" 
"Spectrum c0, c1, c2, c3;\n" 
"TexMap_GetTexel(pixels, width, height, s0, t0, &c0);\n" 
"TexMap_GetTexel(pixels, width, height, s0, t0 + 1, &c1);\n" 
"TexMap_GetTexel(pixels, width, height, s0 + 1, t0, &c2);\n" 
"TexMap_GetTexel(pixels, width, height, s0 + 1, t0 + 1, &c3);\n" 
"const float k0 = ids * idt;\n" 
"const float k1 = ids * dt;\n" 
"const float k2 = ds * idt;\n" 
"const float k3 = ds * dt;\n" 
"col->r = k0 * c0.r + k1 * c1.r + k2 * c2.r + k3 * c3.r;\n" 
"col->g = k0 * c0.g + k1 * c1.g + k2 * c2.g + k3 * c3.g;\n" 
"col->b = k0 * c0.b + k1 * c1.b + k2 * c2.b + k3 * c3.b;\n" 
"}\n" 
"float TexMap_GetAlpha(__global float *alphas, const uint width, const uint height,\n" 
"const float u, const float v) {\n" 
"const float s = u * width - 0.5f;\n" 
"const float t = v * height - 0.5f;\n" 
"const int s0 = (int)floor(s);\n" 
"const int t0 = (int)floor(t);\n" 
"const float ds = s - s0;\n" 
"const float dt = t - t0;\n" 
"const float ids = 1.f - ds;\n" 
"const float idt = 1.f - dt;\n" 
"const float c0 = TexMap_GetAlphaTexel(alphas, width, height, s0, t0);\n" 
"const float c1 = TexMap_GetAlphaTexel(alphas, width, height, s0, t0 + 1);\n" 
"const float c2 = TexMap_GetAlphaTexel(alphas, width, height, s0 + 1, t0);\n" 
"const float c3 = TexMap_GetAlphaTexel(alphas, width, height, s0 + 1, t0 + 1);\n" 
"const float k0 = ids * idt;\n" 
"const float k1 = ids * dt;\n" 
"const float k2 = ds * idt;\n" 
"const float k3 = ds * dt;\n" 
"return k0 * c0 + k1 * c1 + k2 * c2 + k3 * c3;\n" 
"}\n" 
"#if defined(PARAM_HAS_INFINITELIGHT)\n" 
"void InfiniteLight_Le(__global InfiniteLight *infiniteLight, __global Spectrum *infiniteLightMap, Spectrum *le, const Vector *dir) {\n" 
"const float u = 1.f - SphericalPhi(dir) * INV_TWOPI +  infiniteLight->shiftU;\n" 
"const float v = SphericalTheta(dir) * INV_PI + infiniteLight->shiftV;\n" 
"TexMap_GetColor(infiniteLightMap, infiniteLight->width, infiniteLight->height, u, v, le);\n" 
"le->r *= infiniteLight->gain.r;\n" 
"le->g *= infiniteLight->gain.g;\n" 
"le->b *= infiniteLight->gain.b;\n" 
"}\n" 
"#endif\n" 
"#if defined(PARAM_HAS_SUNLIGHT)\n" 
"void SunLight_Le(__global SunLight *sunLight, Spectrum *le, const Vector *dir) {\n" 
"const float cosThetaMax = sunLight->cosThetaMax;\n" 
"Vector sundir = sunLight->sundir;\n" 
"if((cosThetaMax < 1.f) && (Dot(dir, &sundir) > cosThetaMax))\n" 
"*le = sunLight->suncolor;\n" 
"else {\n" 
"le->r = 0.f;\n" 
"le->g = 0.f;\n" 
"le->b = 0.f;\n" 
"}\n" 
"}\n" 
"void SunLight_Sample_L(__global SunLight *sunLight,\n" 
"const Point *hitPoint,\n" 
"float *pdf, Spectrum *f, Ray *shadowRay,\n" 
"const float u0, const float u1) {\n" 
"const float cosThetaMax = sunLight->cosThetaMax;\n" 
"const Vector sundir = sunLight->sundir;\n" 
"const Vector x = sunLight->x;\n" 
"const Vector y = sunLight->y;\n" 
"Vector wi;\n" 
"UniformSampleCone(&wi, u0, u1, cosThetaMax, &x, &y, &sundir);\n" 
"shadowRay->o = *hitPoint;\n" 
"shadowRay->d = wi;\n" 
"shadowRay->mint = PARAM_RAY_EPSILON;\n" 
"shadowRay->maxt = FLT_MAX;\n" 
"*f = sunLight->suncolor;\n" 
"*pdf = UniformConePdf(cosThetaMax);\n" 
"}\n" 
"#endif\n" 
"#if defined(PARAM_HAS_SKYLIGHT)\n" 
"float RiAngleBetween(float thetav, float phiv, float theta, float phi) {\n" 
"const float cospsi = sin(thetav) * sin(theta) * cos(phi - phiv) + cos(thetav) * cos(theta);\n" 
"if (cospsi >= 1.f)\n" 
"return 0.f;\n" 
"if (cospsi <= -1.f)\n" 
"return M_PI;\n" 
"return acos(cospsi);\n" 
"}\n" 
"float SkyLight_PerezBase(__global float *lam, float theta, float gamma) {\n" 
"return (1.f + lam[1] * exp(lam[2] / cos(theta))) *\n" 
"(1.f + lam[3] * exp(lam[4] * gamma)  + lam[5] * cos(gamma) * cos(gamma));\n" 
"}\n" 
"void SkyLight_ChromaticityToSpectrum(const float Y, const float x, const float y, Spectrum *s) {\n" 
"float X, Z;\n" 
"if (y != 0.f)\n" 
"X = (x / y) * Y;\n" 
"else\n" 
"X = 0.f;\n" 
"if (y != 0.f && Y != 0.f)\n" 
"Z = (1.f - x - y) / y * Y;\n" 
"else\n" 
"Z = 0.f;\n" 
"// Assuming sRGB (D65 illuminant)\n" 
"s->r =  3.2410f * X - 1.5374f * Y - 0.4986f * Z;\n" 
"s->g = -0.9692f * X + 1.8760f * Y + 0.0416f * Z;\n" 
"s->b =  0.0556f * X - 0.2040f * Y + 1.0570f * Z;\n" 
"}\n" 
"void SkyLight_GetSkySpectralRadiance(__global SkyLight *skyLight,\n" 
"const float theta, const float phi, Spectrum *spect) {\n" 
"// add bottom half of hemisphere with horizon colour\n" 
"const float theta_fin = min(theta, (const float)((M_PI * 0.5f) - 0.001f));\n" 
"const float gamma = RiAngleBetween(theta, phi, skyLight->thetaS, skyLight->phiS);\n" 
"// Compute xyY values\n" 
"const float x = skyLight->zenith_x * SkyLight_PerezBase(skyLight->perez_x, theta_fin, gamma);\n" 
"const float y = skyLight->zenith_y * SkyLight_PerezBase(skyLight->perez_y, theta_fin, gamma);\n" 
"const float Y = skyLight->zenith_Y * SkyLight_PerezBase(skyLight->perez_Y, theta_fin, gamma);\n" 
"SkyLight_ChromaticityToSpectrum(Y, x, y, spect);\n" 
"}\n" 
"void SkyLight_Le(__global SkyLight *skyLight, Spectrum *f, const Vector *dir) {\n" 
"const float theta = SphericalTheta(dir);\n" 
"const float phi = SphericalPhi(dir);\n" 
"Spectrum s;\n" 
"SkyLight_GetSkySpectralRadiance(skyLight, theta, phi, &s);\n" 
"f->r = skyLight->gain.r * s.r;\n" 
"f->g = skyLight->gain.g * s.g;\n" 
"f->b = skyLight->gain.b * s.b;\n" 
"}\n" 
"#endif\n" 
"void Mesh_InterpolateColor(__global Spectrum *colors, __global Triangle *triangles,\n" 
"const uint triIndex, const float b1, const float b2, Spectrum *C) {\n" 
"__global Triangle *tri = &triangles[triIndex];\n" 
"const float b0 = 1.f - b1 - b2;\n" 
"C->r = b0 * colors[tri->v[0]].r + b1 * colors[tri->v[1]].r + b2 * colors[tri->v[2]].r;\n" 
"C->g = b0 * colors[tri->v[0]].g + b1 * colors[tri->v[1]].g + b2 * colors[tri->v[2]].g;\n" 
"C->b = b0 * colors[tri->v[0]].b + b1 * colors[tri->v[1]].b + b2 * colors[tri->v[2]].b;\n" 
"}\n" 
"void Mesh_InterpolateNormal(__global Vector *normals, __global Triangle *triangles,\n" 
"const uint triIndex, const float b1, const float b2, Vector *N) {\n" 
"__global Triangle *tri = &triangles[triIndex];\n" 
"const float b0 = 1.f - b1 - b2;\n" 
"N->x = b0 * normals[tri->v[0]].x + b1 * normals[tri->v[1]].x + b2 * normals[tri->v[2]].x;\n" 
"N->y = b0 * normals[tri->v[0]].y + b1 * normals[tri->v[1]].y + b2 * normals[tri->v[2]].y;\n" 
"N->z = b0 * normals[tri->v[0]].z + b1 * normals[tri->v[1]].z + b2 * normals[tri->v[2]].z;\n" 
"Normalize(N);\n" 
"}\n" 
"void Mesh_InterpolateUV(__global UV *uvs, __global Triangle *triangles,\n" 
"const uint triIndex, const float b1, const float b2, UV *uv) {\n" 
"__global Triangle *tri = &triangles[triIndex];\n" 
"const float b0 = 1.f - b1 - b2;\n" 
"uv->u = b0 * uvs[tri->v[0]].u + b1 * uvs[tri->v[1]].u + b2 * uvs[tri->v[2]].u;\n" 
"uv->v = b0 * uvs[tri->v[0]].v + b1 * uvs[tri->v[1]].v + b2 * uvs[tri->v[2]].v;\n" 
"}\n" 
"float Mesh_Area(__global Point *verts, __global Triangle *triangles,\n" 
"const uint triIndex) {\n" 
"__global Triangle *tri = &triangles[triIndex];\n" 
"const __global Point *pp0 = &verts[tri->v[0]];\n" 
"const __global Point *pp1 = &verts[tri->v[1]];\n" 
"const __global Point *pp2 = &verts[tri->v[2]];\n" 
"const float4 p0 = (float4)(pp0->x, pp0->y, pp0->z, 0.f);\n" 
"const float4 p1 = (float4)(pp1->x, pp1->y, pp1->z, 0.f);\n" 
"const float4 p2 = (float4)(pp2->x, pp2->y, pp2->z, 0.f);\n" 
"return 0.5f * length(cross(p1 - p0, p2 - p0));\n" 
"}\n" 
"float InstanceMesh_Area(__global float (*m)[4], __global Point *verts,\n" 
"__global Triangle *triangles, const uint triIndex) {\n" 
"__global Triangle *tri = &triangles[triIndex];\n" 
"Point pp0 = verts[tri->v[0]];\n" 
"TransformPoint(m, &pp0);\n" 
"Point pp1 = verts[tri->v[1]];\n" 
"TransformPoint(m, &pp1);\n" 
"Point pp2 = verts[tri->v[2]];\n" 
"TransformPoint(m, &pp2);\n" 
"const float4 p0 = (float4)(pp0.x, pp0.y, pp0.z, 0.f);\n" 
"const float4 p1 = (float4)(pp1.x, pp1.y, pp1.z, 0.f);\n" 
"const float4 p2 = (float4)(pp2.x, pp2.y, pp2.z, 0.f);\n" 
"return 0.5f * length(cross(p1 - p0, p2 - p0));\n" 
"}\n" 
"//------------------------------------------------------------------------------\n" 
"// Materials\n" 
"//------------------------------------------------------------------------------\n" 
"void Matte_Sample_f(__global MatteParam *mat, const Vector *wo, Vector *wi,\n" 
"float *pdf, Spectrum *f, const Vector *shadeN,\n" 
"const float u0, const float u1\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", int *specularBounce\n" 
"#endif\n" 
") {\n" 
"Vector dir;\n" 
"CosineSampleHemisphere(&dir, u0, u1);\n" 
"*pdf = dir.z * INV_PI;\n" 
"Vector v1, v2;\n" 
"CoordinateSystem(shadeN, &v1, &v2);\n" 
"wi->x = v1.x * dir.x + v2.x * dir.y + shadeN->x * dir.z;\n" 
"wi->y = v1.y * dir.x + v2.y * dir.y + shadeN->y * dir.z;\n" 
"wi->z = v1.z * dir.x + v2.z * dir.y + shadeN->z * dir.z;\n" 
"// Using 0.0001 instead of 0.0 to cut down fireflies\n" 
"if (dir.z <= 0.0001f)\n" 
"*pdf = 0.f;\n" 
"else {\n" 
"f->r = mat->r;\n" 
"f->g = mat->g;\n" 
"f->b = mat->b;\n" 
"}\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = FALSE;\n" 
"#endif\n" 
"}\n" 
"void Mirror_Sample_f(__global MirrorParam *mat, const Vector *wo, Vector *wi,\n" 
"float *pdf, Spectrum *f, const Vector *shadeN\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", int *specularBounce\n" 
"#endif\n" 
") {\n" 
"const float k = 2.f * Dot(shadeN, wo);\n" 
"wi->x = k * shadeN->x - wo->x;\n" 
"wi->y = k * shadeN->y - wo->y;\n" 
"wi->z = k * shadeN->z - wo->z;\n" 
"*pdf = 1.f;\n" 
"f->r = mat->r;\n" 
"f->g = mat->g;\n" 
"f->b = mat->b;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->specularBounce;\n" 
"#endif\n" 
"}\n" 
"void Glass_Sample_f(__global GlassParam *mat,\n" 
"const Vector *wo, Vector *wi, float *pdf, Spectrum *f, const Vector *N, const Vector *shadeN,\n" 
"const float u0\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", int *specularBounce\n" 
"#endif\n" 
") {\n" 
"Vector reflDir;\n" 
"const float k = 2.f * Dot(N, wo);\n" 
"reflDir.x = k * N->x - wo->x;\n" 
"reflDir.y = k * N->y - wo->y;\n" 
"reflDir.z = k * N->z - wo->z;\n" 
"// Ray from outside going in ?\n" 
"const bool into = (Dot(N, shadeN) > 0.f);\n" 
"const float nc = mat->ousideIor;\n" 
"const float nt = mat->ior;\n" 
"const float nnt = into ? (nc / nt) : (nt / nc);\n" 
"const float ddn = -Dot(wo, shadeN);\n" 
"const float cos2t = 1.f - nnt * nnt * (1.f - ddn * ddn);\n" 
"// Total internal reflection\n" 
"if (cos2t < 0.f) {\n" 
"*wi = reflDir;\n" 
"*pdf = 1.f;\n" 
"f->r = mat->refl_r;\n" 
"f->g = mat->refl_g;\n" 
"f->b = mat->refl_b;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->reflectionSpecularBounce;\n" 
"#endif\n" 
"} else {\n" 
"const float kk = (into ? 1.f : -1.f) * (ddn * nnt + sqrt(cos2t));\n" 
"Vector nkk = *N;\n" 
"nkk.x *= kk;\n" 
"nkk.y *= kk;\n" 
"nkk.z *= kk;\n" 
"Vector transDir;\n" 
"transDir.x = -nnt * wo->x - nkk.x;\n" 
"transDir.y = -nnt * wo->y - nkk.y;\n" 
"transDir.z = -nnt * wo->z - nkk.z;\n" 
"Normalize(&transDir);\n" 
"const float c = 1.f - (into ? -ddn : Dot(&transDir, N));\n" 
"const float R0 = mat->R0;\n" 
"const float Re = R0 + (1.f - R0) * c * c * c * c * c;\n" 
"const float Tr = 1.f - Re;\n" 
"const float P = .25f + .5f * Re;\n" 
"if (Tr == 0.f) {\n" 
"if (Re == 0.f)\n" 
"*pdf = 0.f;\n" 
"else {\n" 
"*wi = reflDir;\n" 
"*pdf = 1.f;\n" 
"f->r = mat->refl_r;\n" 
"f->g = mat->refl_g;\n" 
"f->b = mat->refl_b;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->reflectionSpecularBounce;\n" 
"#endif\n" 
"}\n" 
"} else if (Re == 0.f) {\n" 
"*wi = transDir;\n" 
"*pdf = 1.f;\n" 
"f->r = mat->refrct_r;\n" 
"f->g = mat->refrct_g;\n" 
"f->b = mat->refrct_b;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->transmitionSpecularBounce;\n" 
"#endif\n" 
"} else if (u0 < P) {\n" 
"*wi = reflDir;\n" 
"*pdf = P / Re;\n" 
"f->r = mat->refl_r / (*pdf);\n" 
"f->g = mat->refl_g / (*pdf);\n" 
"f->b = mat->refl_b / (*pdf);\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->reflectionSpecularBounce;\n" 
"#endif\n" 
"} else {\n" 
"*wi = transDir;\n" 
"*pdf = (1.f - P) / Tr;\n" 
"f->r = mat->refrct_r / (*pdf);\n" 
"f->g = mat->refrct_g / (*pdf);\n" 
"f->b = mat->refrct_b / (*pdf);\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->transmitionSpecularBounce;\n" 
"#endif\n" 
"}\n" 
"}\n" 
"}\n" 
"void MatteMirror_Sample_f(__global MatteMirrorParam *mat, const Vector *wo, Vector *wi,\n" 
"float *pdf, Spectrum *f, const Vector *shadeN,\n" 
"const float u0, const float u1, const float u2\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", int *specularBounce\n" 
"#endif\n" 
") {\n" 
"const float totFilter = mat->totFilter;\n" 
"const float comp = u2 * totFilter;\n" 
"float mpdf;\n" 
"if (comp > mat->matteFilter) {\n" 
"Mirror_Sample_f(&mat->mirror, wo, wi, pdf, f, shadeN\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", specularBounce\n" 
"#endif\n" 
");\n" 
"mpdf = mat->mirrorPdf;\n" 
"} else {\n" 
"Matte_Sample_f(&mat->matte, wo, wi, pdf, f, shadeN, u0, u1\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", specularBounce\n" 
"#endif\n" 
");\n" 
"mpdf = mat->mattePdf;\n" 
"}\n" 
"*pdf *= mpdf;\n" 
"f->r /= mpdf;\n" 
"f->g /= mpdf;\n" 
"f->b /= mpdf;\n" 
"}\n" 
"void GlossyReflection(const Vector *wo, Vector *wi, const float exponent,\n" 
"const Vector *shadeN, const float u0, const float u1) {\n" 
"const float phi = 2.f * M_PI * u0;\n" 
"const float cosTheta = pow(1.f - u1, exponent);\n" 
"const float sinTheta = sqrt(max(0.f, 1.f - cosTheta * cosTheta));\n" 
"const float x = cos(phi) * sinTheta;\n" 
"const float y = sin(phi) * sinTheta;\n" 
"const float z = cosTheta;\n" 
"Vector w;\n" 
"const float RdotShadeN = Dot(shadeN, wo);\n" 
"w.x = (2.f * RdotShadeN) * shadeN->x - wo->x;\n" 
"w.y = (2.f * RdotShadeN) * shadeN->y - wo->y;\n" 
"w.z = (2.f * RdotShadeN) * shadeN->z - wo->z;\n" 
"Vector u, a;\n" 
"if (fabs(shadeN->x) > .1f) {\n" 
"a.x = 0.f;\n" 
"a.y = 1.f;\n" 
"} else {\n" 
"a.x = 1.f;\n" 
"a.y = 0.f;\n" 
"}\n" 
"a.z = 0.f;\n" 
"Cross(&u, &a, &w);\n" 
"Normalize(&u);\n" 
"Vector v;\n" 
"Cross(&v, &w, &u);\n" 
"wi->x = x * u.x + y * v.x + z * w.x;\n" 
"wi->y = x * u.y + y * v.y + z * w.y;\n" 
"wi->z = x * u.z + y * v.z + z * w.z;\n" 
"}\n" 
"void Metal_Sample_f(__global MetalParam *mat, const Vector *wo, Vector *wi,\n" 
"float *pdf, Spectrum *f, const Vector *shadeN,\n" 
"const float u0, const float u1\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", int *specularBounce\n" 
"#endif\n" 
") {\n" 
"GlossyReflection(wo, wi, mat->exponent, shadeN, u0, u1);\n" 
"if (Dot(wi, shadeN) > 0.f) {\n" 
"*pdf = 1.f;\n" 
"f->r = mat->r;\n" 
"f->g = mat->g;\n" 
"f->b = mat->b;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->specularBounce;\n" 
"#endif\n" 
"} else\n" 
"*pdf = 0.f;\n" 
"}\n" 
"void MatteMetal_Sample_f(__global MatteMetalParam *mat, const Vector *wo, Vector *wi,\n" 
"float *pdf, Spectrum *f, const Vector *shadeN,\n" 
"const float u0, const float u1, const float u2\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", int *specularBounce\n" 
"#endif\n" 
") {\n" 
"const float totFilter = mat->totFilter;\n" 
"const float comp = u2 * totFilter;\n" 
"float mpdf;\n" 
"if (comp > mat->matteFilter) {\n" 
"Metal_Sample_f(&mat->metal, wo, wi, pdf, f, shadeN, u0, u1\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", specularBounce\n" 
"#endif\n" 
");\n" 
"mpdf = mat->metalPdf;\n" 
"} else {\n" 
"Matte_Sample_f(&mat->matte, wo, wi, pdf, f, shadeN, u0, u1\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", specularBounce\n" 
"#endif\n" 
");\n" 
"mpdf = mat->mattePdf;\n" 
"}\n" 
"*pdf *= mpdf;\n" 
"f->r /= mpdf;\n" 
"f->g /= mpdf;\n" 
"f->b /= mpdf;\n" 
"}\n" 
"void Alloy_Sample_f(__global AlloyParam *mat, const Vector *wo, Vector *wi,\n" 
"float *pdf, Spectrum *f, const Vector *shadeN,\n" 
"const float u0, const float u1, const float u2\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", int *specularBounce\n" 
"#endif\n" 
") {\n" 
"// Schilick's approximation\n" 
"const float c = 1.f - Dot(wo, shadeN);\n" 
"const float R0 = mat->R0;\n" 
"const float Re = R0 + (1.f - R0) * c * c * c * c * c;\n" 
"const float P = .25f + .5f * Re;\n" 
"if (u2 <= P) {\n" 
"GlossyReflection(wo, wi, mat->exponent, shadeN, u0, u1);\n" 
"*pdf = P / Re;\n" 
"f->r = mat->refl_r / (*pdf);\n" 
"f->g = mat->refl_g / (*pdf);\n" 
"f->b = mat->refl_b / (*pdf);\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->specularBounce;\n" 
"#endif\n" 
"} else {\n" 
"Vector dir;\n" 
"CosineSampleHemisphere(&dir, u0, u1);\n" 
"*pdf = dir.z * INV_PI;\n" 
"Vector v1, v2;\n" 
"CoordinateSystem(shadeN, &v1, &v2);\n" 
"wi->x = v1.x * dir.x + v2.x * dir.y + shadeN->x * dir.z;\n" 
"wi->y = v1.y * dir.x + v2.y * dir.y + shadeN->y * dir.z;\n" 
"wi->z = v1.z * dir.x + v2.z * dir.y + shadeN->z * dir.z;\n" 
"// Using 0.0001 instead of 0.0 to cut down fireflies\n" 
"if (dir.z <= 0.0001f)\n" 
"*pdf = 0.f;\n" 
"else {\n" 
"const float iRe = 1.f - Re;\n" 
"const float k = (1.f - P) / iRe;\n" 
"*pdf *= k;\n" 
"f->r = mat->diff_r / k;\n" 
"f->g = mat->diff_g / k;\n" 
"f->b = mat->diff_b / k;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = FALSE;\n" 
"#endif\n" 
"}\n" 
"}\n" 
"}\n" 
"void ArchGlass_Sample_f(__global ArchGlassParam *mat,\n" 
"const Vector *wo, Vector *wi, float *pdf, Spectrum *f, const Vector *N, const Vector *shadeN,\n" 
"const float u0\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
", int *specularBounce\n" 
"#endif\n" 
") {\n" 
"// Ray from outside going in ?\n" 
"const bool into = (Dot(N, shadeN) > 0.f);\n" 
"if (!into) {\n" 
"// No internal reflections\n" 
"wi->x = -wo->x;\n" 
"wi->y = -wo->y;\n" 
"wi->z = -wo->z;\n" 
"*pdf = 1.f;\n" 
"f->r = mat->refrct_r;\n" 
"f->g = mat->refrct_g;\n" 
"f->b = mat->refrct_b;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->transmitionSpecularBounce;\n" 
"#endif\n" 
"} else {\n" 
"// RR to choose if reflect the ray or go trough the glass\n" 
"const float comp = u0 * mat->totFilter;\n" 
"if (comp > mat->transFilter) {\n" 
"const float k = 2.f * Dot(N, wo);\n" 
"wi->x = k * N->x - wo->x;\n" 
"wi->y = k * N->y - wo->y;\n" 
"wi->z = k * N->z - wo->z;\n" 
"*pdf = mat->reflPdf;\n" 
"f->r = mat->refl_r / mat->reflPdf;\n" 
"f->g = mat->refl_g / mat->reflPdf;\n" 
"f->b = mat->refl_b / mat->reflPdf;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->reflectionSpecularBounce;\n" 
"#endif\n" 
"} else {\n" 
"wi->x = -wo->x;\n" 
"wi->y = -wo->y;\n" 
"wi->z = -wo->z;\n" 
"*pdf = mat->transPdf;\n" 
"f->r = mat->refrct_r / mat->transPdf;\n" 
"f->g = mat->refrct_g / mat->transPdf;\n" 
"f->b = mat->refrct_b / mat->transPdf;\n" 
"#if defined(PARAM_DIRECT_LIGHT_SAMPLING) || (PARAM_MAX_DIFFUSE_PATH_VERTEX_COUNT < PARAM_MAX_PATH_DEPTH)\n" 
"*specularBounce = mat->transmitionSpecularBounce;\n" 
"#endif\n" 
"}\n" 
"}\n" 
"}\n" 
"//------------------------------------------------------------------------------\n" 
"// Lights\n" 
"//------------------------------------------------------------------------------\n" 
"#if (PARAM_DL_LIGHT_COUNT > 0)\n" 
"void AreaLight_Le(__global AreaLightParam *mat, const Vector *wo, const Vector *lightN, Spectrum *Le) {\n" 
"const bool brightSide = (Dot(lightN, wo) > 0.f);\n" 
"Le->r = brightSide ? mat->gain_r : 0.f;\n" 
"Le->g = brightSide ? mat->gain_g : 0.f;\n" 
"Le->b = brightSide ? mat->gain_b : 0.f;\n" 
"}\n" 
"void SampleTriangleLight(__global TriangleLight *light,	const float u0, const float u1, Point *p) {\n" 
"Point v0, v1, v2;\n" 
"v0 = light->v0;\n" 
"v1 = light->v1;\n" 
"v2 = light->v2;\n" 
"// UniformSampleTriangle(u0, u1, b0, b1);\n" 
"const float su1 = sqrt(u0);\n" 
"const float b0 = 1.f - su1;\n" 
"const float b1 = u1 * su1;\n" 
"const float b2 = 1.f - b0 - b1;\n" 
"p->x = b0 * v0.x + b1 * v1.x + b2 * v2.x;\n" 
"p->y = b0 * v0.y + b1 * v1.y + b2 * v2.y;\n" 
"p->z = b0 * v0.z + b1 * v1.z + b2 * v2.z;\n" 
"}\n" 
"void TriangleLight_Sample_L(__global TriangleLight *l,\n" 
"const Point *hitPoint,\n" 
"float *pdf, Spectrum *f, Ray *shadowRay,\n" 
"const float u0, const float u1) {\n" 
"Point samplePoint;\n" 
"SampleTriangleLight(l, u0, u1, &samplePoint);\n" 
"shadowRay->d.x = samplePoint.x - hitPoint->x;\n" 
"shadowRay->d.y = samplePoint.y - hitPoint->y;\n" 
"shadowRay->d.z = samplePoint.z - hitPoint->z;\n" 
"const float distanceSquared = Dot(&shadowRay->d, &shadowRay->d);\n" 
"const float distance = sqrt(distanceSquared);\n" 
"const float invDistance = 1.f / distance;\n" 
"shadowRay->d.x *= invDistance;\n" 
"shadowRay->d.y *= invDistance;\n" 
"shadowRay->d.z *= invDistance;\n" 
"Vector sampleN = l->normal;\n" 
"const float sampleNdotMinusWi = -Dot(&sampleN, &shadowRay->d);\n" 
"if (sampleNdotMinusWi <= 0.f)\n" 
"*pdf = 0.f;\n" 
"else {\n" 
"*pdf = distanceSquared / (sampleNdotMinusWi * l->area);\n" 
"// Using 0.1 instead of 0.0 to cut down fireflies\n" 
"if (*pdf <= 0.1f)\n" 
"*pdf = 0.f;\n" 
"else {\n" 
"shadowRay->o = *hitPoint;\n" 
"shadowRay->mint = PARAM_RAY_EPSILON;\n" 
"shadowRay->maxt = distance - PARAM_RAY_EPSILON;\n" 
"f->r = l->gain_r;\n" 
"f->g = l->gain_g;\n" 
"f->b = l->gain_b;\n" 
"}\n" 
"}\n" 
"}\n" 
"#endif\n" 
"//------------------------------------------------------------------------------\n" 
"// GenerateCameraRay\n" 
"//------------------------------------------------------------------------------\n" 
"void GenerateCameraRay(\n" 
"__global Sample *sample,\n" 
"__global Ray *ray\n" 
"#if (PARAM_SAMPLER_TYPE == 0)\n" 
", Seed *seed\n" 
"#endif\n" 
", __global Camera *camera) {\n" 
"#if (PARAM_SAMPLER_TYPE == 0) || (PARAM_SAMPLER_TYPE == 1) || (PARAM_SAMPLER_TYPE == 3)\n" 
"__global float *sampleData = &sample->u[0];\n" 
"const uint pixelIndex = sample->pixelIndex;\n" 
"const float scrSampleX = sampleData[IDX_SCREEN_X];\n" 
"const float scrSampleY = sampleData[IDX_SCREEN_Y];\n" 
"const float screenX = pixelIndex % PARAM_IMAGE_WIDTH + scrSampleX - .5f;\n" 
"const float screenY = pixelIndex / PARAM_IMAGE_WIDTH + scrSampleY - .5f;\n" 
"#elif (PARAM_SAMPLER_TYPE == 2)\n" 
"__global float *sampleData = &sample->u[sample->proposed][0];\n" 
"const float screenX = min(sampleData[IDX_SCREEN_X] * PARAM_IMAGE_WIDTH, (float)(PARAM_IMAGE_WIDTH - 1));\n" 
"const float screenY = min(sampleData[IDX_SCREEN_Y] * PARAM_IMAGE_HEIGHT, (float)(PARAM_IMAGE_HEIGHT - 1));\n" 
"#endif\n" 
"Point Pras;\n" 
"Pras.x = screenX;\n" 
"Pras.y = PARAM_IMAGE_HEIGHT - screenY - 1.f;\n" 
"Pras.z = 0;\n" 
"Point orig;\n" 
"// RasterToCamera(Pras, &orig);\n" 
"const float iw = 1.f / (camera->rasterToCameraMatrix[3][0] * Pras.x + camera->rasterToCameraMatrix[3][1] * Pras.y + camera->rasterToCameraMatrix[3][2] * Pras.z + camera->rasterToCameraMatrix[3][3]);\n" 
"orig.x = (camera->rasterToCameraMatrix[0][0] * Pras.x + camera->rasterToCameraMatrix[0][1] * Pras.y + camera->rasterToCameraMatrix[0][2] * Pras.z + camera->rasterToCameraMatrix[0][3]) * iw;\n" 
"orig.y = (camera->rasterToCameraMatrix[1][0] * Pras.x + camera->rasterToCameraMatrix[1][1] * Pras.y + camera->rasterToCameraMatrix[1][2] * Pras.z + camera->rasterToCameraMatrix[1][3]) * iw;\n" 
"orig.z = (camera->rasterToCameraMatrix[2][0] * Pras.x + camera->rasterToCameraMatrix[2][1] * Pras.y + camera->rasterToCameraMatrix[2][2] * Pras.z + camera->rasterToCameraMatrix[2][3]) * iw;\n" 
"Vector dir;\n" 
"dir.x = orig.x;\n" 
"dir.y = orig.y;\n" 
"dir.z = orig.z;\n" 
"const float hither = camera->hither;\n" 
"#if defined(PARAM_CAMERA_HAS_DOF)\n" 
"#if (PARAM_SAMPLER_TYPE == 0)\n" 
"const float dofSampleX = RndFloatValue(seed);\n" 
"const float dofSampleY = RndFloatValue(seed);\n" 
"#elif (PARAM_SAMPLER_TYPE == 1) || (PARAM_SAMPLER_TYPE == 2) || (PARAM_SAMPLER_TYPE == 3)\n" 
"const float dofSampleX = sampleData[IDX_DOF_X];\n" 
"const float dofSampleY = sampleData[IDX_DOF_Y];\n" 
"#endif\n" 
"// Sample point on lens\n" 
"float lensU, lensV;\n" 
"ConcentricSampleDisk(dofSampleX, dofSampleY, &lensU, &lensV);\n" 
"const float lensRadius = camera->lensRadius;\n" 
"lensU *= lensRadius;\n" 
"lensV *= lensRadius;\n" 
"// Compute point on plane of focus\n" 
"const float focalDistance = camera->focalDistance;\n" 
"const float dist = focalDistance - hither;\n" 
"const float ft = dist / dir.z;\n" 
"Point Pfocus;\n" 
"Pfocus.x = orig.x + dir.x * ft;\n" 
"Pfocus.y = orig.y + dir.y * ft;\n" 
"Pfocus.z = orig.z + dir.z * ft;\n" 
"// Update ray for effect of lens\n" 
"const float k = dist / focalDistance;\n" 
"orig.x += lensU * k;\n" 
"orig.y += lensV * k;\n" 
"dir.x = Pfocus.x - orig.x;\n" 
"dir.y = Pfocus.y - orig.y;\n" 
"dir.z = Pfocus.z - orig.z;\n" 
"#endif\n" 
"Normalize(&dir);\n" 
"// CameraToWorld(*ray, ray);\n" 
"Point torig;\n" 
"const float iw2 = 1.f / (camera->cameraToWorldMatrix[3][0] * orig.x + camera->cameraToWorldMatrix[3][1] * orig.y + camera->cameraToWorldMatrix[3][2] * orig.z + camera->cameraToWorldMatrix[3][3]);\n" 
"torig.x = (camera->cameraToWorldMatrix[0][0] * orig.x + camera->cameraToWorldMatrix[0][1] * orig.y + camera->cameraToWorldMatrix[0][2] * orig.z + camera->cameraToWorldMatrix[0][3]) * iw2;\n" 
"torig.y = (camera->cameraToWorldMatrix[1][0] * orig.x + camera->cameraToWorldMatrix[1][1] * orig.y + camera->cameraToWorldMatrix[1][2] * orig.z + camera->cameraToWorldMatrix[1][3]) * iw2;\n" 
"torig.z = (camera->cameraToWorldMatrix[2][0] * orig.x + camera->cameraToWorldMatrix[2][1] * orig.y + camera->cameraToWorldMatrix[2][2] * orig.z + camera->cameraToWorldMatrix[2][3]) * iw2;\n" 
"Vector tdir;\n" 
"tdir.x = camera->cameraToWorldMatrix[0][0] * dir.x + camera->cameraToWorldMatrix[0][1] * dir.y + camera->cameraToWorldMatrix[0][2] * dir.z;\n" 
"tdir.y = camera->cameraToWorldMatrix[1][0] * dir.x + camera->cameraToWorldMatrix[1][1] * dir.y + camera->cameraToWorldMatrix[1][2] * dir.z;\n" 
"tdir.z = camera->cameraToWorldMatrix[2][0] * dir.x + camera->cameraToWorldMatrix[2][1] * dir.y + camera->cameraToWorldMatrix[2][2] * dir.z;\n" 
"ray->o = torig;\n" 
"ray->d = tdir;\n" 
"ray->mint = PARAM_RAY_EPSILON;\n" 
"ray->maxt = (camera->yon - hither) / dir.z;\n" 
"/*printf(\"(%f, %f, %f) (%f, %f, %f) [%f, %f]\\n\",\n" 
"ray->o.x, ray->o.y, ray->o.z, ray->d.x, ray->d.y, ray->d.z,\n" 
"ray->mint, ray->maxt);*/\n" 
"}\n" 
; 
