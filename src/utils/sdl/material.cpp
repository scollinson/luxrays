/***************************************************************************
 *   Copyright (C) 1998-2010 by authors (see AUTHORS.txt )                 *
 *                                                                         *
 *   This file is part of LuxRays.                                         *
 *                                                                         *
 *   LuxRays is free software; you can redistribute it and/or modify       *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 3 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   LuxRays is distributed in the hope that it will be useful,            *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 *                                                                         *
 *   LuxRays website: http://www.luxrender.net                             *
 ***************************************************************************/

#include <boost/lexical_cast.hpp>

#include "luxrays/core/geometry/frame.h"
#include "luxrays/utils/sdl/material.h"
#include "luxrays/utils/sdl/bsdf.h"

using namespace luxrays;
using namespace luxrays::sdl;

//------------------------------------------------------------------------------
// MaterialDefinitions
//------------------------------------------------------------------------------

MaterialDefinitions::MaterialDefinitions() { }

MaterialDefinitions::~MaterialDefinitions() {
	for (std::vector<Material *>::const_iterator it = mats.begin(); it != mats.end(); ++it)
		delete (*it);
}

void MaterialDefinitions::DefineMaterial(const std::string &name, Material *m) {
	if (IsMaterialDefined(name))
		throw std::runtime_error("Already defined material: " + name);

	mats.push_back(m);
	matsByName.insert(std::make_pair(name, m));
}

void MaterialDefinitions::UpdateMaterial(const std::string &name, Material *m) {
	if (!IsMaterialDefined(name))
		throw std::runtime_error("Can not update an undefined material: " + name);

	Material *oldMat = GetMaterial(name);

	// Update name/material definition
	const u_int index = GetMaterialIndex(name);
	mats[index] = m;
	matsByName.erase(name);
	matsByName.insert(std::make_pair(name, m));

	// Delete old material
	delete oldMat;

	// Update all possible reference to old material with the new one
	for (u_int i = 0; i < mats.size(); ++i)
		mats[i]->UpdateMaterialReference(oldMat, m);
}

Material *MaterialDefinitions::GetMaterial(const std::string &name) {
	// Check if the material has been already defined
	std::map<std::string, Material *>::const_iterator it = matsByName.find(name);

	if (it == matsByName.end())
		throw std::runtime_error("Reference to an undefined material: " + name);
	else
		return it->second;
}

u_int MaterialDefinitions::GetMaterialIndex(const std::string &name) {
	return GetMaterialIndex(GetMaterial(name));
}

u_int MaterialDefinitions::GetMaterialIndex(const Material *m) const {
	for (u_int i = 0; i < mats.size(); ++i) {
		if (m == mats[i])
			return i;
	}

	throw std::runtime_error("Reference to an undefined material: " + boost::lexical_cast<std::string>(m));
}

std::vector<std::string> MaterialDefinitions::GetMaterialNames() const {
	std::vector<std::string> names;
	names.reserve(mats.size());
	for (std::map<std::string, Material *>::const_iterator it = matsByName.begin(); it != matsByName.end(); ++it)
		names.push_back(it->first);

	return names;
}

void MaterialDefinitions::DeleteMaterial(const std::string &name) {
	const u_int index = GetMaterialIndex(name);
	mats.erase(mats.begin() + index);
	matsByName.erase(name);
}

//------------------------------------------------------------------------------
// Matte material
//------------------------------------------------------------------------------

Spectrum MatteMaterial::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	if (directPdfW)
		*directPdfW = fabsf((hitPoint.fromLight ? localEyeDir.z : localLightDir.z) * INV_PI);

	if (reversePdfW)
		*reversePdfW = fabsf((hitPoint.fromLight ? localLightDir.z : localEyeDir.z) * INV_PI);

	*event = DIFFUSE | REFLECT;
	return Kd->GetColorValue(hitPoint).Clamp() * INV_PI;
}

Spectrum MatteMaterial::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	if (fabsf(localFixedDir.z) < DEFAULT_COS_EPSILON_STATIC)
		return Spectrum();

	*localSampledDir = Sgn(localFixedDir.z) * CosineSampleHemisphere(u0, u1, pdfW);

	*absCosSampledDir = fabsf(localSampledDir->z);
	if (*absCosSampledDir < DEFAULT_COS_EPSILON_STATIC)
		return Spectrum();

	*event = DIFFUSE | REFLECT;
	return Kd->GetColorValue(hitPoint).Clamp() * INV_PI;
}

void MatteMaterial::Pdf(const HitPoint &hitPoint,
		const Vector &localLightDir, const Vector &localEyeDir,
		float *directPdfW, float *reversePdfW) const {
	if (directPdfW)
		*directPdfW = fabsf((hitPoint.fromLight ? localEyeDir.z : localLightDir.z) * INV_PI);

	if (reversePdfW)
		*reversePdfW = fabsf((hitPoint.fromLight ? localLightDir.z : localEyeDir.z) * INV_PI);
}

void MatteMaterial::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	Kd->AddReferencedTextures(referencedTexs);
}

Properties MatteMaterial::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "matte");
	props.SetString("scene.materials." + name + ".kd", Kd->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// Mirror material
//------------------------------------------------------------------------------

Spectrum MirrorMaterial::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	return Spectrum();
}

Spectrum MirrorMaterial::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	*event = SPECULAR | REFLECT;

	*localSampledDir = Vector(-localFixedDir.x, -localFixedDir.y, localFixedDir.z);
	*pdfW = 1.f;

	*absCosSampledDir = fabsf(localSampledDir->z);
	// The absCosSampledDir is used to compensate the other one used inside the integrator
	return Kr->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
}

void MirrorMaterial::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	Kr->AddReferencedTextures(referencedTexs);
}

Properties MirrorMaterial::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "mirror");
	props.SetString("scene.materials." + name + ".kr", Kr->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// Glass material
//------------------------------------------------------------------------------

Spectrum GlassMaterial::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	return Spectrum();
}

Spectrum GlassMaterial::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	// Ray from outside going in ?
	const bool into = (localFixedDir.z > 0.f);

	// TODO: remove
	const Vector shadeN(0.f, 0.f, into ? 1.f : -1.f);
	const Vector N(0.f, 0.f, 1.f);

	const Vector rayDir = -localFixedDir;
	const Vector reflDir = rayDir - (2.f * Dot(N, rayDir)) * Vector(N);

	const float nc = ousideIor->GetGreyValue(hitPoint);
	const float nt = ior->GetGreyValue(hitPoint);
	const float nnt = into ? (nc / nt) : (nt / nc);
	const float nnt2 = nnt * nnt;
	const float ddn = Dot(rayDir, shadeN);
	const float cos2t = 1.f - nnt2 * (1.f - ddn * ddn);

	// Total internal reflection
	if (cos2t < 0.f) {
		*event = SPECULAR | REFLECT;
		*localSampledDir = reflDir;
		*absCosSampledDir = fabsf(localSampledDir->z);
		*pdfW = 1.f;

		// The absCosSampledDir is used to compensate the other one used inside the integrator
		return Kr->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	}

	const float kk = (into ? 1.f : -1.f) * (ddn * nnt + sqrtf(cos2t));
	const Vector nkk = kk * Vector(N);
	const Vector transDir = Normalize(nnt * rayDir - nkk);

	const float c = 1.f - (into ? -ddn : Dot(transDir, N));
	const float c2 = c * c;
	const float a = nt - nc;
	const float b = nt + nc;
	const float R0 = a * a / (b * b);
	const float Re = R0 + (1.f - R0) * c2 * c2 * c;
	const float Tr = 1.f - Re;
	const float P = .25f + .5f * Re;

	if (Tr == 0.f) {
		if (Re == 0.f)
			return Spectrum();
		else {
			*event = SPECULAR | REFLECT;
			*localSampledDir = reflDir;
			*absCosSampledDir = fabsf(localSampledDir->z);
			*pdfW = 1.f;

			// The absCosSampledDir is used to compensate the other one used inside the integrator
			return Kr->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
		}
	} else if (Re == 0.f) {
		*event = SPECULAR | TRANSMIT;
		*localSampledDir = transDir;
		*absCosSampledDir = fabsf(localSampledDir->z);
		*pdfW = 1.f;

		if (hitPoint.fromLight)
			return Kt->GetColorValue(hitPoint).Clamp() * (nnt2 / (*absCosSampledDir));
		else
			return Kt->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	} else if (passThroughEvent < P) {
		*event = SPECULAR | REFLECT;
		*localSampledDir = reflDir;
		*absCosSampledDir = fabsf(localSampledDir->z);
		*pdfW = P / Re;

		// The absCosSampledDir is used to compensate the other one used inside the integrator
		return Kr->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	} else {
		*event = SPECULAR | TRANSMIT;
		*localSampledDir = transDir;
		*absCosSampledDir = fabsf(localSampledDir->z);
		*pdfW = (1.f - P) / Tr;

		// The absCosSampledDir is used to compensate the other one used inside the integrator
		if (hitPoint.fromLight)
			return Kt->GetColorValue(hitPoint).Clamp() * (nnt2 / (*absCosSampledDir));
		else
			return Kt->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	}
}

void GlassMaterial::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	Kr->AddReferencedTextures(referencedTexs);
	Kt->AddReferencedTextures(referencedTexs);
	ousideIor->AddReferencedTextures(referencedTexs);
	ior->AddReferencedTextures(referencedTexs);
}

Properties GlassMaterial::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "glass");
	props.SetString("scene.materials." + name + ".kr", Kr->GetName());
	props.SetString("scene.materials." + name + ".kt", Kt->GetName());
	props.SetString("scene.materials." + name + ".ioroutside", ousideIor->GetName());
	props.SetString("scene.materials." + name + ".iorinside", ior->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// Architectural glass material
//------------------------------------------------------------------------------

Spectrum ArchGlassMaterial::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	return Spectrum();
}

Spectrum ArchGlassMaterial::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	// Ray from outside going in ?
	const bool into = (localFixedDir.z > 0.f);

	// TODO: remove
	const Vector shadeN(0.f, 0.f, into ? 1.f : -1.f);
	const Vector N(0.f, 0.f, 1.f);

	const Vector rayDir = -localFixedDir;
	const Vector reflDir = rayDir - (2.f * Dot(N, rayDir)) * Vector(N);

	const float nc = ousideIor->GetGreyValue(hitPoint);
	const float nt = ior->GetGreyValue(hitPoint);
	const float nnt = into ? (nc / nt) : (nt / nc);
	const float nnt2 = nnt * nnt;
	const float ddn = Dot(rayDir, shadeN);
	const float cos2t = 1.f - nnt2 * (1.f - ddn * ddn);

	// Total internal reflection
	if (cos2t < 0.f) {
		// Architectural glass reflect only from the outside
		if (!into)
			return Spectrum();

		*event = SPECULAR | REFLECT;
		*localSampledDir = reflDir;
		*absCosSampledDir = fabsf(localSampledDir->z);
		*pdfW = 1.f;

		// The absCosSampledDir is used to compensate the other one used inside the integrator
		return Kr->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	}

	const float kk = (into ? 1.f : -1.f) * (ddn * nnt + sqrtf(cos2t));
	const Vector nkk = kk * Vector(N);
	const Vector transDir = Normalize(nnt * rayDir - nkk);

	const float c = 1.f - (into ? -ddn : Dot(transDir, N));
	const float c2 = c * c;
	const float a = nt - nc;
	const float b = nt + nc;
	const float R0 = a * a / (b * b);
	const float Re = R0 + (1.f - R0) * c2 * c2 * c;
	const float Tr = 1.f - Re;
	const float P = .25f + .5f * Re;

	if (Tr == 0.f) {
		if (Re == 0.f)
			return Spectrum();
		else {
			// Architectural glass reflect only from the outside
			if (!into)
				return Spectrum();

			*event = SPECULAR | REFLECT;
			*localSampledDir = reflDir;
			*absCosSampledDir = fabsf(localSampledDir->z);
			*pdfW = 1.f;

			// The absCosSampledDir is used to compensate the other one used inside the integrator
			return Kr->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
		}
	} else if (Re == 0.f) {
		*event = SPECULAR | TRANSMIT;
		*localSampledDir = transDir;
		*absCosSampledDir = fabsf(localSampledDir->z);
		*pdfW = 1.f;

		if (hitPoint.fromLight)
			return Kt->GetColorValue(hitPoint).Clamp() * (nnt2 / (*absCosSampledDir));
		else
			return Kt->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	} else if (passThroughEvent < P) {
		// Architectural glass reflect only from the outside
		if (!into)
			return Spectrum();

		*event = SPECULAR | REFLECT;
		*localSampledDir = reflDir;
		*absCosSampledDir = fabsf(localSampledDir->z);
		*pdfW = P / Re;

		// The absCosSampledDir is used to compensate the other one used inside the integrator
		return Kr->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	} else {
		*event = SPECULAR | TRANSMIT;
		*localSampledDir = transDir;
		*absCosSampledDir = fabsf(localSampledDir->z);
		*pdfW = (1.f - P) / Tr;

		// The absCosSampledDir is used to compensate the other one used inside the integrator
		if (hitPoint.fromLight)
			return Kt->GetColorValue(hitPoint).Clamp() * (nnt2 / (*absCosSampledDir));
		else
			return Kt->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	}
}

Spectrum ArchGlassMaterial::GetPassThroughTransparency(const HitPoint &hitPoint,
		const Vector &localFixedDir, const float passThroughEvent) const {
	// Ray from outside going in ?
	const bool into = (localFixedDir.z > 0.f);

	// TODO: remove
	const Vector shadeN(0.f, 0.f, into ? 1.f : -1.f);
	const Vector N(0.f, 0.f, 1.f);

	const Vector rayDir = -localFixedDir;

	const float nc = ousideIor->GetGreyValue(hitPoint);
	const float nt = ior->GetGreyValue(hitPoint);
	const float nnt = into ? (nc / nt) : (nt / nc);
	const float nnt2 = nnt * nnt;
	const float ddn = Dot(rayDir, shadeN);
	const float cos2t = 1.f - nnt2 * (1.f - ddn * ddn);

	// Total internal reflection
	if (cos2t < 0.f)
		return Spectrum();

	const float kk = (into ? 1.f : -1.f) * (ddn * nnt + sqrtf(cos2t));
	const Vector nkk = kk * Vector(N);
	const Vector transDir = Normalize(nnt * rayDir - nkk);

	const float c = 1.f - (into ? -ddn : Dot(transDir, N));
	const float c2 = c * c;
	const float a = nt - nc;
	const float b = nt + nc;
	const float R0 = a * a / (b * b);
	const float Re = R0 + (1.f - R0) * c2 * c2 * c;
	const float Tr = 1.f - Re;
	const float P = .25f + .5f * Re;

	if (Tr == 0.f) {
		return Spectrum();
	} else if (Re == 0.f) {
		if (hitPoint.fromLight)
			return Kt->GetColorValue(hitPoint).Clamp() * nnt2;
		else
			return Kt->GetColorValue(hitPoint).Clamp();
	} else if (passThroughEvent < P) {
		return Spectrum();
	} else {
		if (hitPoint.fromLight)
			return Kt->GetColorValue(hitPoint).Clamp() * nnt2;
		else
			return Kt->GetColorValue(hitPoint).Clamp();
	}
}

void ArchGlassMaterial::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	Kr->AddReferencedTextures(referencedTexs);
	Kt->AddReferencedTextures(referencedTexs);
	ousideIor->AddReferencedTextures(referencedTexs);
	ior->AddReferencedTextures(referencedTexs);
}

Properties ArchGlassMaterial::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "archglass");
	props.SetString("scene.materials." + name + ".kr", Kr->GetName());
	props.SetString("scene.materials." + name + ".kt", Kt->GetName());
	props.SetString("scene.materials." + name + ".ioroutside", ousideIor->GetName());
	props.SetString("scene.materials." + name + ".iorinside", ior->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// Metal material
//------------------------------------------------------------------------------

Spectrum MetalMaterial::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	return Spectrum();
}

Vector MetalMaterial::GlossyReflection(const Vector &localFixedDir, const float exponent,
		const float u0, const float u1) {
	// Ray from outside going in ?
	const bool into = (localFixedDir.z > 0.f);
	const Vector shadeN(0.f, 0.f, into ? 1.f : -1.f);

	const float phi = 2.f * M_PI * u0;
	const float cosTheta = powf(1.f - u1, exponent);
	const float sinTheta = sqrtf(Max(0.f, 1.f - cosTheta * cosTheta));
	const float x = cosf(phi) * sinTheta;
	const float y = sinf(phi) * sinTheta;
	const float z = cosTheta;

	const Vector dir = -localFixedDir;
	const float dp = Dot(shadeN, dir);
	const Vector w = dir - (2.f * dp) * Vector(shadeN);

	const Vector u = Normalize(Cross(
			(fabsf(shadeN.x) > .1f) ? Vector(0.f, 1.f, 0.f) :  Vector(1.f, 0.f, 0.f),
			w));
	const Vector v = Cross(w, u);

	return x * u + y * v + z * w;
}

Spectrum MetalMaterial::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	const float e = 1.f / (Max(exponent->GetGreyValue(hitPoint), 0.f) + 1.f);
	*localSampledDir = GlossyReflection(localFixedDir, e, u0, u1);

	if (localSampledDir->z * localFixedDir.z > 0.f) {
		*event = SPECULAR | REFLECT;
		*pdfW = 1.f;
		*absCosSampledDir = fabsf(localSampledDir->z);
		// The absCosSampledDir is used to compensate the other one used inside the integrator
		return Kr->GetColorValue(hitPoint).Clamp() / (*absCosSampledDir);
	} else
		return Spectrum();
}

void MetalMaterial::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	Kr->AddReferencedTextures(referencedTexs);
	exponent->AddReferencedTextures(referencedTexs);
}

Properties MetalMaterial::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "metal");
	props.SetString("scene.materials." + name + ".kr", Kr->GetName());
	props.SetString("scene.materials." + name + ".exp", exponent->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// Mix material
//------------------------------------------------------------------------------

Spectrum MixMaterial::GetPassThroughTransparency(const HitPoint &hitPoint,
		const Vector &localFixedDir, const float passThroughEvent) const {
	const float weight2 = Clamp(mixFactor->GetGreyValue(hitPoint), 0.f, 1.f);
	const float weight1 = 1.f - weight2;

	if (passThroughEvent < weight1)
		return matA->GetPassThroughTransparency(hitPoint, localFixedDir, passThroughEvent / weight1);
	else
		return matB->GetPassThroughTransparency(hitPoint, localFixedDir, (passThroughEvent - weight2) / weight2);
}

Spectrum MixMaterial::GetEmittedRadiance(const HitPoint &hitPoint) const {
	Spectrum result;

	const float weight2 = Clamp(mixFactor->GetGreyValue(hitPoint), 0.f, 1.f);
	const float weight1 = 1.f - weight2;

	if (matA->IsLightSource() && (weight1 > 0.f))
		result += weight1 * matA->GetEmittedRadiance(hitPoint);
	if (matB->IsLightSource() && (weight2 > 0.f))
		result += weight2 * matB->GetEmittedRadiance(hitPoint);

	return result;
}

Spectrum MixMaterial::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	Spectrum result;

	const float weight2 = Clamp(mixFactor->GetGreyValue(hitPoint), 0.f, 1.f);
	const float weight1 = 1.f - weight2;

	if (directPdfW)
		*directPdfW = 0.f;
	if (reversePdfW)
		*reversePdfW = 0.f;

	BSDFEvent eventMatA = NONE;
	if (weight1 > 0.f) {
		float directPdfWMatA, reversePdfWMatA;
		const Spectrum matAResult = matA->Evaluate(hitPoint, localLightDir, localEyeDir, &eventMatA, &directPdfWMatA, &reversePdfWMatA);
		if (!matAResult.Black()) {
			result += weight1 * matAResult;

			if (directPdfW)
				*directPdfW += weight1 * directPdfWMatA;
			if (reversePdfW)
				*reversePdfW += weight1 * reversePdfWMatA;
		}
	}

	BSDFEvent eventMatB = NONE;
	if (weight2 > 0.f) {
		float directPdfWMatB, reversePdfWMatB;
		const Spectrum matBResult = matB->Evaluate(hitPoint, localLightDir, localEyeDir, &eventMatB, &directPdfWMatB, &reversePdfWMatB);
		if (!matBResult.Black()) {
			result += weight2 * matBResult;

			if (directPdfW)
				*directPdfW += weight2 * directPdfWMatB;
			if (reversePdfW)
				*reversePdfW += weight2 * reversePdfWMatB;
		}
	}

	*event = eventMatA | eventMatB;

	return result;
}

Spectrum MixMaterial::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	const float weight2 = Clamp(mixFactor->GetGreyValue(hitPoint), 0.f, 1.f);
	const float weight1 = 1.f - weight2;

	const bool sampleMatA = (passThroughEvent < weight1);

	const float weightFirst = sampleMatA ? weight1 : weight2;
	const float weightSecond = sampleMatA ? weight2 : weight1;

	const float passThroughEventFirst = sampleMatA ? (passThroughEvent / weight1) : (passThroughEvent - weight1) / weight2;

	// Sample the first material, evaluate the second
	const Material *matFirst = sampleMatA ? matA : matB;
	const Material *matSecond = sampleMatA ? matB : matA;

	// Sample the first material
	Spectrum result = matFirst->Sample(hitPoint, localFixedDir, localSampledDir,
			u0, u1, passThroughEventFirst, pdfW, absCosSampledDir, event);
	if (result.Black())
		return Spectrum();
	*pdfW *= weightFirst;
	result *= weightFirst;

	// Evaluate the second material
	const Vector &localLightDir = (hitPoint.fromLight) ? localFixedDir : *localSampledDir;
	const Vector &localEyeDir = (hitPoint.fromLight) ? *localSampledDir : localFixedDir;
	BSDFEvent eventSecond;
	float pdfWSecond;
	Spectrum evalSecond = matSecond->Evaluate(hitPoint, localLightDir, localEyeDir, &eventSecond, &pdfWSecond);
	if (!evalSecond.Black()) {
		result += weightSecond * evalSecond;
		*pdfW += weightSecond * pdfWSecond;
	}

	return result;
}

void MixMaterial::Pdf(const HitPoint &hitPoint,
		const Vector &localLightDir, const Vector &localEyeDir,
		float *directPdfW, float *reversePdfW) const {
	const float weight2 = Clamp(mixFactor->GetGreyValue(hitPoint), 0.f, 1.f);
	const float weight1 = 1.f - weight2;

	float directPdfWMatA = 1.f;
	float reversePdfWMatA = 1.f;
	if (weight1 > 0.f)
		matA->Pdf(hitPoint, localLightDir, localEyeDir, &directPdfWMatA, &reversePdfWMatA);

	float directPdfWMatB = 1.f;
	float reversePdfWMatB = 1.f;
	if (weight2 > 0.f)
		matB->Pdf(hitPoint, localLightDir, localEyeDir, &directPdfWMatB, &reversePdfWMatB);

	if (directPdfW)
		*directPdfW = weight1 * directPdfWMatA + weight2 *directPdfWMatB;
	if (reversePdfW)
		*reversePdfW = weight1 * reversePdfWMatA + weight2 * reversePdfWMatB;
}

void MixMaterial::UpdateMaterialReference(const Material *oldMat,  const Material *newMat) {
	if (matA == oldMat)
		matA = newMat;

	if (matB == oldMat)
		matB = newMat;
}

bool MixMaterial::IsReferencing(const Material *mat) const {
	if (matA == mat)
		return true;
	if (matB == mat)
		return true;

	const MixMaterial *mixA = dynamic_cast<const MixMaterial *>(matA);
	if (mixA && mixA->IsReferencing(mat))
		return true;
	const MixMaterial *mixB = dynamic_cast<const MixMaterial *>(matB);
	if (mixB && mixB->IsReferencing(mat))
		return true;

	return false;
}

void MixMaterial::AddReferencedMaterials(std::set<const Material *> &referencedMats) const {
	Material::AddReferencedMaterials(referencedMats);

	referencedMats.insert(matA);
	matA->AddReferencedMaterials(referencedMats);

	referencedMats.insert(matB);
	matB->AddReferencedMaterials(referencedMats);
}

void MixMaterial::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	matA->AddReferencedTextures(referencedTexs);
	matB->AddReferencedTextures(referencedTexs);
	mixFactor->AddReferencedTextures(referencedTexs);
}

Properties MixMaterial::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "mix");
	props.SetString("scene.materials." + name + ".material1", matA->GetName());
	props.SetString("scene.materials." + name + ".material2", matB->GetName());
	props.SetString("scene.materials." + name + ".amount", mixFactor->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// Null material
//------------------------------------------------------------------------------

Spectrum NullMaterial::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
		return Spectrum();
}

Spectrum NullMaterial::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	//throw std::runtime_error("Internal error, called NullMaterial::Sample()");

	*localSampledDir = -localFixedDir;
	*absCosSampledDir = 1.f;

	*pdfW = 1.f;
	*event = SPECULAR | TRANSMIT;
	return Spectrum(1.f);
}

Properties NullMaterial::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "null");
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// MatteTranslucent material
//------------------------------------------------------------------------------

Spectrum MatteTranslucentMaterial::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	const float absCosSampledDir = Dot(localLightDir, localEyeDir);

	const Spectrum r = Kr->GetColorValue(hitPoint).Clamp();
	const Spectrum t = Kt->GetColorValue(hitPoint).Clamp() * 
		// Energy conservation
		(Spectrum(1.f) - r);

	if (directPdfW)
		*directPdfW = .5f * fabsf((hitPoint.fromLight ? localEyeDir.z : localLightDir.z) * (.5f * INV_PI));

	if (reversePdfW)
		*reversePdfW = .5f * fabsf((hitPoint.fromLight ? localLightDir.z : localEyeDir.z) * (.5f * INV_PI));

	if (absCosSampledDir > 0.f) {
		*event = DIFFUSE | REFLECT;
		return r * INV_PI;
	} else {
		*event = DIFFUSE | TRANSMIT;
		return t * INV_PI;
	}
}

Spectrum MatteTranslucentMaterial::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	if (fabsf(localFixedDir.z) < DEFAULT_COS_EPSILON_STATIC)
		return Spectrum();

	*localSampledDir = CosineSampleHemisphere(u0, u1, pdfW);
	*absCosSampledDir = fabsf(localSampledDir->z);
	if (*absCosSampledDir < DEFAULT_COS_EPSILON_STATIC)
		return Spectrum();

	*pdfW *= .5f;

	const Spectrum r = Kr->GetColorValue(hitPoint).Clamp();
	const Spectrum t = Kt->GetColorValue(hitPoint).Clamp() * 
		// Energy conservation
		(Spectrum(1.f) - r);

	if (passThroughEvent < .5f) {
		*localSampledDir *= Sgn(localFixedDir.z);
		*event = DIFFUSE | REFLECT;
		return r * INV_PI;
	} else {
		*localSampledDir *= -Sgn(localFixedDir.z);
		*event = DIFFUSE | TRANSMIT;
		return t * INV_PI;
	}
}

void MatteTranslucentMaterial::Pdf(const HitPoint &hitPoint,
		const Vector &localLightDir, const Vector &localEyeDir,
		float *directPdfW, float *reversePdfW) const {
	if (directPdfW)
		*directPdfW = fabsf((hitPoint.fromLight ? localEyeDir.z : localLightDir.z) * (.5f * INV_PI));

	if (reversePdfW)
		*reversePdfW = fabsf((hitPoint.fromLight ? localLightDir.z : localEyeDir.z) * (.5f * INV_PI));
}

void MatteTranslucentMaterial::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	Kr->AddReferencedTextures(referencedTexs);
	Kt->AddReferencedTextures(referencedTexs);
}

Properties MatteTranslucentMaterial::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "mattetranslucent");
	props.SetString("scene.materials." + name + ".kr", Kr->GetName());
	props.SetString("scene.materials." + name + ".kt", Kt->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// Glossy2 material
//
// LuxRender Glossy2 material porting.
//------------------------------------------------------------------------------

float Glossy2Material::SchlickBSDF_CoatingWeight(const Spectrum &ks, const Vector &localFixedDir) const {
	// No sampling on the back face
	if (localFixedDir.z <= 0.f)
		return 0.f;

	// Approximate H by using reflection direction for wi
	const float u = fabsf(localFixedDir.z);
	const Spectrum S = FresnelSlick_Evaluate(ks, u);

	// Ensures coating is never sampled less than half the time
	// unless we are on the back face
	return .5f * (1.f + S.Filter());
}

Spectrum Glossy2Material::SchlickBSDF_CoatingF(const Spectrum &ks, const float roughness,
		const float anisotropy, const Vector &localFixedDir,	const Vector &localSampledDir) const {
	// No sampling on the back face
	if (localFixedDir.z <= 0.f)
		return Spectrum();

	const float coso = fabsf(localFixedDir.z);
	const float cosi = fabsf(localSampledDir.z);

	const Vector wh(Normalize(localFixedDir + localSampledDir));
	const Spectrum S = FresnelSlick_Evaluate(ks, AbsDot(localSampledDir, wh));

	const float G = SchlickDistribution_G(roughness, localFixedDir, localSampledDir);

	// Multibounce - alternative with interreflection in the coating creases
	const float factor = SchlickDistribution_D(roughness, wh, anisotropy) * G / (4.f * cosi) + 
		(multibounce ? coso * Clamp((1.f - G) / (4.f * cosi * coso), 0.f, 1.f) : 0.f);

	return factor * S;
}

Spectrum Glossy2Material::SchlickBSDF_CoatingAbsorption(const float cosi, const float coso,
		const Spectrum &alpha, const float depth) const {
	if (depth > 0.f) {
		// 1/cosi+1/coso=(cosi+coso)/(cosi*coso)
		const float depthFactor = depth * (cosi + coso) / (cosi * coso);
		return Exp(alpha * -depthFactor);
	} else
		return Spectrum(1.f);
}

Spectrum Glossy2Material::SchlickBSDF_CoatingSampleF(const bool fromLight, const Spectrum ks,
		const float roughness, const float anisotropy, const Vector &localFixedDir, Vector *localSampledDir,
		float u0, float u1, float *pdf) const {
	// No sampling on the back face
	if (localFixedDir.z <= 0.f)
		return Spectrum();

	Vector wh;
	float d, specPdf;
	SchlickDistribution_SampleH(roughness, anisotropy, u0, u1, &wh, &d, &specPdf);
	const float cosWH = Dot(localFixedDir, wh);
	*localSampledDir = 2.f * cosWH * wh - localFixedDir;

	if ((localSampledDir->z < DEFAULT_COS_EPSILON_STATIC) || (localFixedDir.z * localSampledDir->z < 0.f))
		return Spectrum();

	const float coso = fabsf(localFixedDir.z);
	const float cosi = fabsf(localSampledDir->z);

	*pdf = specPdf / (4.f * cosWH);
	if (*pdf <= 0.f)
		return Spectrum();

	Spectrum S = FresnelSlick_Evaluate(ks, cosWH);

	const float G = SchlickDistribution_G(roughness, localFixedDir, *localSampledDir);
	if (!fromLight)
		//CoatingF(sw, *wi, wo, f_);
		S *= d * G / (4.f * coso) + 
				(multibounce ? cosi * Clamp((1.f - G) / (4.f * coso * cosi), 0.f, 1.f) : 0.f);
	else
		//CoatingF(sw, wo, *wi, f_);
		S *= d * G / (4.f * cosi) + 
				(multibounce ? coso * Clamp((1.f - G) / (4.f * cosi * coso), 0.f, 1.f) : 0.f);

	return S;
}

float Glossy2Material::SchlickBSDF_CoatingPdf(const float roughness, const float anisotropy,
		const Vector &localFixedDir, const Vector &localSampledDir) const {
	// No sampling on the back face
	if (localFixedDir.z <= 0.f)
		return 0.f;

	const Vector wh(Normalize(localFixedDir + localSampledDir));
	return SchlickDistribution_Pdf(roughness, wh, anisotropy) / (4.f * AbsDot(localFixedDir, wh));
}

Spectrum Glossy2Material::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	const Vector &localFixedDir = hitPoint.fromLight ? localLightDir : localEyeDir;
	const Vector &localSampledDir = hitPoint.fromLight ? localEyeDir : localLightDir;

	const Spectrum baseF = Kd->GetColorValue(hitPoint).Clamp() * INV_PI;
	if (localEyeDir.z <= 0.f) {
		// Back face: no coating

		if (directPdfW)
			*directPdfW = fabsf(localSampledDir.z * INV_PI);

		if (reversePdfW)
			*reversePdfW = fabsf(localFixedDir.z * INV_PI);

		*event = DIFFUSE | REFLECT;
		return baseF;
	}

	// Front face: coating+base
	*event = GLOSSY | REFLECT;

	Spectrum ks = Ks->GetColorValue(hitPoint);
	const float i = index->GetGreyValue(hitPoint);
	if (i > 0.f) {
		const float ti = (i - 1.f) / (i + 1.f);
		ks *= ti * ti;
	}
	ks = ks.Clamp();

	const float u = Clamp(nu->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float v = Clamp(nv->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float u2 = u * u;
	const float v2 = v * v;
	const float anisotropy = (u2 < v2) ? (1.f - u2 / v2) : (v2 / u2 - 1.f);
	const float roughness = u * v;

	if (directPdfW) {
		const float wCoating = SchlickBSDF_CoatingWeight(ks, localFixedDir);
		const float wBase = 1.f - wCoating;

		*directPdfW = wBase * fabsf(localSampledDir.z * INV_PI) +
				wCoating * SchlickBSDF_CoatingPdf(roughness, anisotropy, localFixedDir, localSampledDir);
	}

	if (reversePdfW) {
		const float wCoatingR = SchlickBSDF_CoatingWeight(ks, localSampledDir);
		const float wBaseR = 1.f - wCoatingR;

		*reversePdfW = wBaseR * fabsf(localFixedDir.z * INV_PI) +
				wCoatingR * SchlickBSDF_CoatingPdf(roughness, anisotropy, localSampledDir, localFixedDir);
	}

	// Absorption
	const float cosi = fabsf(localSampledDir.z);
	const float coso = fabsf(localFixedDir.z);
	const Spectrum alpha = Ka->GetColorValue(hitPoint).Clamp();
	const float d = depth->GetGreyValue(hitPoint);
	const Spectrum absorption = SchlickBSDF_CoatingAbsorption(cosi, coso, alpha, d);

	// Coating fresnel factor
	const Vector H(Normalize(localFixedDir + localSampledDir));
	const Spectrum S = FresnelSlick_Evaluate(ks, AbsDot(localSampledDir, H));

	const Spectrum coatingF = SchlickBSDF_CoatingF(ks, roughness, anisotropy, localFixedDir, localSampledDir);

	// Blend in base layer Schlick style
	// assumes coating bxdf takes fresnel factor S into account
	
	// The cosi is used to compensate the other one used inside the integrator
	return coatingF / cosi + absorption * (Spectrum(1.f) - S) * baseF;
}

Spectrum Glossy2Material::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	if (fabsf(localFixedDir.z) < DEFAULT_COS_EPSILON_STATIC)
		return Spectrum();

	Spectrum ks = Ks->GetColorValue(hitPoint);
	const float i = index->GetGreyValue(hitPoint);
	if (i > 0.f) {
		const float ti = (i - 1.f) / (i + 1.f);
		ks *= ti * ti;
	}
	ks = ks.Clamp();

	const float u = Clamp(nu->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float v = Clamp(nv->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float u2 = u * u;
	const float v2 = v * v;
	const float anisotropy = (u2 < v2) ? (1.f - u2 / v2) : (v2 / u2 - 1.f);
	const float roughness = u * v;

	// Coating is used only on the front face
	const float wCoating = (localFixedDir.z <= 0.f) ? 0.f : SchlickBSDF_CoatingWeight(ks, localFixedDir);
	const float wBase = 1.f - wCoating;

	float basePdf, coatingPdf;
	Spectrum baseF, coatingF;

	if (passThroughEvent < wBase) {
		// Sample base BSDF (Matte BSDF)
		*localSampledDir = Sgn(localFixedDir.z) * CosineSampleHemisphere(u0, u1, &basePdf);

		*absCosSampledDir = fabsf(localSampledDir->z);
		if (*absCosSampledDir < DEFAULT_COS_EPSILON_STATIC)
			return Spectrum();

		baseF = Kd->GetColorValue(hitPoint).Clamp() * INV_PI;

		// Evaluate coating BSDF (Schlick BSDF)
		coatingF = SchlickBSDF_CoatingF(ks, roughness, anisotropy, localFixedDir, *localSampledDir);
		coatingPdf = SchlickBSDF_CoatingPdf(roughness, anisotropy, localFixedDir, *localSampledDir);

		*event = DIFFUSE | REFLECT;
	} else {
		// Sample coating BSDF (Schlick BSDF)
		coatingF = SchlickBSDF_CoatingSampleF(hitPoint.fromLight, ks, roughness, anisotropy,
				localFixedDir, localSampledDir, u0, u1, &coatingPdf);
		if (coatingF.Black())
			return Spectrum();

		*absCosSampledDir = fabsf(localSampledDir->z);
		if (*absCosSampledDir < DEFAULT_COS_EPSILON_STATIC)
			return Spectrum();

		// Evaluate base BSDF (Matte BSDF)
		basePdf = fabsf((hitPoint.fromLight ? localFixedDir.z : localSampledDir->z) * INV_PI);
		baseF = Kd->GetColorValue(hitPoint).Clamp() * INV_PI;

		*event = GLOSSY | REFLECT;
	}

	*pdfW = coatingPdf * wCoating + basePdf * wBase;
	if (localFixedDir.z > 0.f) {
		// Front face reflection: coating+base

		// Absorption
		const float cosi = fabsf(localSampledDir->z);
		const float coso = fabsf(localFixedDir.z);
		const Spectrum alpha = Ka->GetColorValue(hitPoint).Clamp();
		const float d = depth->GetGreyValue(hitPoint);
		const Spectrum absorption = SchlickBSDF_CoatingAbsorption(cosi, coso, alpha, d);

		// Coating fresnel factor
		const Vector H(Normalize(localFixedDir + *localSampledDir));
		const Spectrum S = FresnelSlick_Evaluate(ks, AbsDot(*localSampledDir, H));

		// Blend in base layer Schlick style
		// coatingF already takes fresnel factor S into account
		
		// The cosi is used to compensate the other one used inside the integrator
		return coatingF / cosi + absorption * (Spectrum(1.f) - S) * baseF;
	} else {
		// Back face reflection: base

		return baseF;
	}
}

void Glossy2Material::Pdf(const HitPoint &hitPoint,
		const Vector &localLightDir, const Vector &localEyeDir,
		float *directPdfW, float *reversePdfW) const {
	const Vector &localFixedDir = hitPoint.fromLight ? localLightDir : localEyeDir;
	const Vector &localSampledDir = hitPoint.fromLight ? localEyeDir : localLightDir;

	Spectrum ks = Ks->GetColorValue(hitPoint);
	const float i = index->GetGreyValue(hitPoint);
	if (i > 0.f) {
		const float ti = (i - 1.f) / (i + 1.f);
		ks *= ti * ti;
	}
	ks = ks.Clamp();

	const float u = Clamp(nu->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float v = Clamp(nv->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float u2 = u * u;
	const float v2 = v * v;
	const float anisotropy = (u2 < v2) ? (1.f - u2 / v2) : (v2 / u2 - 1.f);
	const float roughness = u * v;

	if (directPdfW) {
		const float wCoating = SchlickBSDF_CoatingWeight(ks, localFixedDir);
		const float wBase = 1.f - wCoating;

		*directPdfW = wBase * fabsf(localSampledDir.z * INV_PI) +
				wCoating * SchlickBSDF_CoatingPdf(roughness, anisotropy, localFixedDir, localSampledDir);
	}

	if (reversePdfW) {
		const float wCoatingR = SchlickBSDF_CoatingWeight(ks, localSampledDir);
		const float wBaseR = 1.f - wCoatingR;

		*reversePdfW = wBaseR * fabsf(localFixedDir.z * INV_PI) +
				wCoatingR * SchlickBSDF_CoatingPdf(roughness, anisotropy, localSampledDir, localFixedDir);
	}
}

void Glossy2Material::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	Kd->AddReferencedTextures(referencedTexs);
	Ks->AddReferencedTextures(referencedTexs);
	nu->AddReferencedTextures(referencedTexs);
	nv->AddReferencedTextures(referencedTexs);
	Ka->AddReferencedTextures(referencedTexs);
	depth->AddReferencedTextures(referencedTexs);
	index->AddReferencedTextures(referencedTexs);
}

Properties Glossy2Material::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "glossy2");
	props.SetString("scene.materials." + name + ".kd", Kd->GetName());
	props.SetString("scene.materials." + name + ".ks", Ks->GetName());
	props.SetString("scene.materials." + name + ".uroughness", nu->GetName());
	props.SetString("scene.materials." + name + ".vroughness", nv->GetName());
	props.SetString("scene.materials." + name + ".ka", Ka->GetName());
	props.SetString("scene.materials." + name + ".d", depth->GetName());
	props.SetString("scene.materials." + name + ".index", index->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// Metal2 material
//
// LuxRender Metal2 material porting.
//------------------------------------------------------------------------------

Spectrum Metal2Material::Evaluate(const HitPoint &hitPoint,
	const Vector &localLightDir, const Vector &localEyeDir, BSDFEvent *event,
	float *directPdfW, float *reversePdfW) const {
	const Vector &localFixedDir = hitPoint.fromLight ? localLightDir : localEyeDir;
	const Vector &localSampledDir = hitPoint.fromLight ? localEyeDir : localLightDir;

	const float u = Clamp(nu->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float v = Clamp(nv->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float u2 = u * u;
	const float v2 = v * v;
	const float anisotropy = (u2 < v2) ? (1.f - u2 / v2) : (v2 / u2 - 1.f);
	const float roughness = u * v;

	const Vector wh(Normalize(localFixedDir + localSampledDir));
	const float cosWH = Dot(localFixedDir, wh);

	if (directPdfW)
		*directPdfW = SchlickDistribution_Pdf(roughness, wh, anisotropy) / (4.f * AbsDot(localFixedDir, wh));

	if (reversePdfW)
		*reversePdfW = SchlickDistribution_Pdf(roughness, wh, anisotropy) / (4.f * AbsDot(localSampledDir, wh));

	const Spectrum etaVal = n->GetColorValue(hitPoint);
	const Spectrum kVal = k->GetColorValue(hitPoint);
	const Spectrum F = FresnelGeneral_Evaluate(etaVal, kVal, cosWH);

	const float G = SchlickDistribution_G(roughness, localFixedDir, localSampledDir);

	const float cosi = fabsf(localSampledDir.z);
	const float factor = SchlickDistribution_D(roughness, wh, anisotropy) * G / (4.f * cosi);

	*event = GLOSSY | REFLECT;

	// The absCosSampledDir is used to compensate the other one used inside the integrator
	const float absCosSampledDir = fabsf(localSampledDir.z);
	return (factor / absCosSampledDir) * F;
}

Spectrum Metal2Material::Sample(const HitPoint &hitPoint,
	const Vector &localFixedDir, Vector *localSampledDir,
	const float u0, const float u1, const float passThroughEvent,
	float *pdfW, float *absCosSampledDir, BSDFEvent *event) const {
	if (fabsf(localFixedDir.z) < DEFAULT_COS_EPSILON_STATIC)
		return Spectrum();

	const float u = Clamp(nu->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float v = Clamp(nv->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float u2 = u * u;
	const float v2 = v * v;
	const float anisotropy = (u2 < v2) ? (1.f - u2 / v2) : (v2 / u2 - 1.f);
	const float roughness = u * v;

	Vector wh;
	float d, specPdf;
	SchlickDistribution_SampleH(roughness, anisotropy, u0, u1, &wh, &d, &specPdf);
	const float cosWH = Dot(localFixedDir, wh);
	*localSampledDir = 2.f * cosWH * wh - localFixedDir;

	const float coso = fabsf(localFixedDir.z);
	const float cosi = fabsf(localSampledDir->z);
	*absCosSampledDir = cosi;
	if ((*absCosSampledDir < DEFAULT_COS_EPSILON_STATIC) || (localFixedDir.z * localSampledDir->z < 0.f))
		return Spectrum();

	*pdfW = specPdf / (4.f * cosWH);
	if (*pdfW <= 0.f)
		return Spectrum();

	const float G = SchlickDistribution_G(roughness, localFixedDir, *localSampledDir);

	const Spectrum etaVal = n->GetColorValue(hitPoint);
	const Spectrum kVal = k->GetColorValue(hitPoint);
	Spectrum F = FresnelGeneral_Evaluate(etaVal, kVal, cosWH);

	const float factor = d * G;
	F *= factor;
	if (!hitPoint.fromLight)
		F /= 4.f * coso;
	else
		F /= 4.f * cosi;

	*event = GLOSSY | REFLECT;
	// The cosi is used to compensate the other one used inside the integrator
	return F / cosi;
}

void Metal2Material::Pdf(const HitPoint &hitPoint,
		const Vector &localLightDir, const Vector &localEyeDir,
		float *directPdfW, float *reversePdfW) const {
	const Vector &localFixedDir = hitPoint.fromLight ? localLightDir : localEyeDir;
	const Vector &localSampledDir = hitPoint.fromLight ? localEyeDir : localLightDir;

	const float u = Clamp(nu->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float v = Clamp(nv->GetGreyValue(hitPoint), 6e-3f, 1.f);
	const float u2 = u * u;
	const float v2 = v * v;
	const float anisotropy = (u2 < v2) ? (1.f - u2 / v2) : (v2 / u2 - 1.f);
	const float roughness = u * v;

	const Vector wh(Normalize(localFixedDir + localSampledDir));

	if (directPdfW)
		*directPdfW = SchlickDistribution_Pdf(roughness, wh, anisotropy) / (4.f * AbsDot(localFixedDir, wh));

	if (reversePdfW)
		*reversePdfW = SchlickDistribution_Pdf(roughness, wh, anisotropy) / (4.f * AbsDot(localSampledDir, wh));
}

void Metal2Material::AddReferencedTextures(std::set<const Texture *> &referencedTexs) const {
	Material::AddReferencedTextures(referencedTexs);

	n->AddReferencedTextures(referencedTexs);
	k->AddReferencedTextures(referencedTexs);
	nu->AddReferencedTextures(referencedTexs);
	nv->AddReferencedTextures(referencedTexs);
}

Properties Metal2Material::ToProperties() const  {
	Properties props;

	const std::string name = GetName();
	props.SetString("scene.materials." + name + ".type", "metal2");
	props.SetString("scene.materials." + name + ".n", n->GetName());
	props.SetString("scene.materials." + name + ".k", k->GetName());
	props.SetString("scene.materials." + name + ".uroughness", nu->GetName());
	props.SetString("scene.materials." + name + ".vroughness", nv->GetName());
	props.Load(Material::ToProperties());

	return props;
}

//------------------------------------------------------------------------------
// SchlickDistribution
//------------------------------------------------------------------------------

namespace luxrays { namespace sdl {

float SchlickDistribution_SchlickZ(const float roughness, const float cosNH) {
	const float d = 1.f + (roughness - 1) * cosNH * cosNH;
	return (roughness > 0.f) ? (roughness / (d * d)) : INFINITY;
}

float SchlickDistribution_SchlickA(const Vector &H, const float anisotropy) {
	const float h = sqrtf(H.x * H.x + H.y * H.y);
	if (h > 0.f) {
		const float w = (anisotropy > 0.f ? H.x : H.y) / h;
		const float p = 1.f - fabsf(anisotropy);
		return sqrtf(p / (p * p + w * w * (1.f - p * p)));
	}

	return 1.f;
}

float SchlickDistribution_D(const float roughness, const Vector &wh,
		const float anisotropy) {
	const float cosTheta = fabsf(wh.z);
	return SchlickDistribution_SchlickZ(roughness, cosTheta) * SchlickDistribution_SchlickA(wh, anisotropy) * INV_PI;
}

float SchlickDistribution_SchlickG(const float roughness,
		const float costheta) {
	return costheta / (costheta * (1.f - roughness) + roughness);
}

float SchlickDistribution_G(const float roughness, const Vector &localFixedDir,
	const Vector &localSampledDir) {
	return SchlickDistribution_SchlickG(roughness, fabsf(localFixedDir.z)) *
			SchlickDistribution_SchlickG(roughness, fabsf(localSampledDir.z));
}

static float GetPhi(const float a, const float b) {
	return M_PI * .5f * sqrtf(a * b / (1.f - a * (1.f - b)));
}

void SchlickDistribution_SampleH(const float roughness, const float anisotropy,
		const float u0, const float u1, Vector *wh, float *d, float *pdf) {
	float u1x4 = u1 * 4.f;
	const float cos2Theta = u0 / (roughness * (1 - u0) + u0);
	const float cosTheta = sqrtf(cos2Theta);
	const float sinTheta = sqrtf(1.f - cos2Theta);
	const float p = 1.f - fabsf(anisotropy);
	float phi;
	if (u1x4 < 1.f) {
		phi = GetPhi(u1x4 * u1x4, p * p);
	} else if (u1x4 < 2.f) {
		u1x4 = 2.f - u1x4;
		phi = M_PI - GetPhi(u1x4 * u1x4, p * p);
	} else if (u1x4 < 3.f) {
		u1x4 -= 2.f;
		phi = M_PI + GetPhi(u1x4 * u1x4, p * p);
	} else {
		u1x4 = 4.f - u1x4;
		phi = M_PI * 2.f - GetPhi(u1x4 * u1x4, p * p);
	}

	if (anisotropy > 0.f)
		phi += M_PI * .5f;

	*wh = Vector(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
	*d = SchlickDistribution_SchlickZ(roughness, cosTheta) * SchlickDistribution_SchlickA(*wh, anisotropy) * INV_PI;
	*pdf = *d;
}

float SchlickDistribution_Pdf(const float roughness, const Vector &wh,
		const float anisotropy) {
	return SchlickDistribution_D(roughness, wh, anisotropy);
}

} }

//------------------------------------------------------------------------------
// FresnelSlick BSDF
//------------------------------------------------------------------------------

namespace luxrays { namespace sdl {

Spectrum FresnelSlick_Evaluate(const Spectrum &normalIncidence, const float cosi) {
	return normalIncidence + (Spectrum(1.f) - normalIncidence) *
		powf(1.f - cosi, 5.f);
}

} }

//------------------------------------------------------------------------------
// FresnelGeneral material
//------------------------------------------------------------------------------

namespace luxrays { namespace sdl {
	
//static Spectrum FrDiel2(const float cosi, const Spectrum &cost,
//		const Spectrum &eta) {
//	Spectrum Rparl(eta * cosi);
//	Rparl = (cost - Rparl) / (cost + Rparl);
//	Spectrum Rperp(eta * cost);
//	Rperp = (Spectrum(cosi) - Rperp) / (Spectrum(cosi) + Rperp);
//
//	return (Rparl * Rparl + Rperp * Rperp) * .5f;
//}
//
//static Spectrum FrDiel(const float cosi, const float cost,
//		const Spectrum &etai, const Spectrum &etat) {
//	return FrDiel2(cosi, Spectrum(cost), etat / etai);
//}
//
//static Spectrum FrCond(const float cosi, const Spectrum &eta,
//		const Spectrum &k) {
//	const Spectrum tmp = (eta * eta + k*k) * (cosi * cosi) + 1;
//	const Spectrum Rparl2 = 
//		(tmp - (2.f * eta * cosi)) /
//		(tmp + (2.f * eta * cosi));
//	const Spectrum tmp_f = eta * eta + k*k + (cosi * cosi);
//	const Spectrum Rperp2 =
//		(tmp_f - (2.f * eta * cosi)) /
//		(tmp_f + (2.f * eta * cosi));
//	return (Rparl2 + Rperp2) * .5f;
//}

static Spectrum FrFull(const float cosi, const Spectrum &cost, const Spectrum &eta, const Spectrum &k) {
	const Spectrum tmp = (eta * eta + k * k) * (cosi * cosi) + (cost * cost);
	const Spectrum Rparl2 = (tmp - (2.f * cosi * cost) * eta) /
		(tmp + (2.f * cosi * cost) * eta);
	const Spectrum tmp_f = (eta * eta + k * k) * (cost * cost) + (cosi * cosi);
	const Spectrum Rperp2 = (tmp_f - (2.f * cosi * cost) * eta) /
		(tmp_f + (2.f * cosi * cost) * eta);
	return (Rparl2 + Rperp2) * .5f;
}

Spectrum FresnelGeneral_Evaluate(const Spectrum &eta, const Spectrum &k, const float cosi) {
	Spectrum sint2(Max(0.f, 1.f - cosi * cosi));
	if (cosi > 0.f)
		sint2 /= eta * eta;
	else
		sint2 *= eta * eta;
	sint2 = sint2.Clamp();

	const Spectrum cost2 = (Spectrum(1.f) - sint2);
	if (cosi > 0.f) {
		const Spectrum a(2.f * k * k * sint2);
		return FrFull(cosi, Sqrt((cost2 + Sqrt(cost2 * cost2 + a * a)) / 2.f), eta, k);
	} else {
		const Spectrum a(2.f * k * k * sint2);
		const Spectrum d2 = eta * eta + k * k;
		return FrFull(-cosi, Sqrt((cost2 + Sqrt(cost2 * cost2 + a * a)) / 2.f), eta / d2, -k / d2);
	}
}

} }
