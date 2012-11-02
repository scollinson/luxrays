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

#include "renderconfig.h"
#include "lightcpu/lightcpu.h"
#include "luxrays/core/geometry/transform.h"
#include "luxrays/utils/core/randomgen.h"

//------------------------------------------------------------------------------
// LightCPU RenderThread
//------------------------------------------------------------------------------

void LightCPURenderEngine::ConnectToEye(Film *film, const float u0,
		const Vector &eyeDir, const float eyeDistance, const Point &lensPoint,
		const Normal &shadeN, const Spectrum &bsdfEval,
		const Spectrum &flux) {
	Scene *scene = renderConfig->scene;

	if (!bsdfEval.Black()) {
		Ray eyeRay(lensPoint, eyeDir);
		eyeRay.maxt = eyeDistance - MachineEpsilon::E(eyeDistance);

		float scrX, scrY;
		if (scene->camera->GetSamplePosition(lensPoint, eyeDir, eyeDistance, &scrX, &scrY)) {
			RayHit eyeRayHit;
			BSDF bsdf;
			Spectrum connectionThroughput;
			if (!scene->Intersect(true, true, u0, &eyeRay, &eyeRayHit, &bsdf, &connectionThroughput)) {
				// Nothing was hit, the light path vertex is visible

				const float cosToCamera = Dot(shadeN, -eyeDir);
				const float cosAtCamera = Dot(scene->camera->GetDir(), eyeDir);

				const float cameraPdfW = 1.f / (cosAtCamera * cosAtCamera * cosAtCamera *
					scene->camera->GetPixelArea());
				const float cameraPdfA = PdfWtoA(cameraPdfW, eyeDistance, cosToCamera);
				const float fluxToRadianceFactor = cameraPdfA;

				film->SplatFiltered(PER_SCREEN_NORMALIZED, scrX, scrY,
						connectionThroughput * flux * fluxToRadianceFactor * bsdfEval);
			}
		}
	}
}

void LightCPURenderEngine::ConnectToEye(Film *film, const float u0,
		const BSDF &bsdf, const Point &lensPoint, const Spectrum &flux) {
	Vector eyeDir(bsdf.hitPoint - lensPoint);
	const float eyeDistance = eyeDir.Length();
	eyeDir /= eyeDistance;

	BSDFEvent event;
	Spectrum bsdfEval = bsdf.Evaluate(-eyeDir, &event);

	ConnectToEye(film, u0, eyeDir, eyeDistance, lensPoint, bsdf.shadeN, bsdfEval, flux);
}

void LightCPURenderEngine::RenderThreadFuncImpl(CPURenderThread *renderThread) {
	//SLG_LOG("[LightCPURenderThread::" << renderThread->threadIndex << "] Rendering thread started");

	//--------------------------------------------------------------------------
	// Initialization
	//--------------------------------------------------------------------------

	LightCPURenderEngine *renderEngine = (LightCPURenderEngine *)renderThread->renderEngine;
	RandomGenerator *rndGen = new RandomGenerator(renderThread->threadIndex + renderThread->seed);
	Scene *scene = renderEngine->renderConfig->scene;
	PerspectiveCamera *camera = scene->camera;
	Film *film = renderThread->threadFilm;
	const unsigned int filmWidth = film->GetWidth();
	const unsigned int filmHeight = film->GetHeight();

	//--------------------------------------------------------------------------
	// Trace light paths
	//--------------------------------------------------------------------------

	renderEngine->threadSamplesCount[renderThread->threadIndex] = 0.0;
	while (!boost::this_thread::interruption_requested()) {
		film->AddSampleCount(PER_SCREEN_NORMALIZED, 1.0);
		renderEngine->threadSamplesCount[renderThread->threadIndex] += 1;

		// Select one light source
		float lightPickPdf;
		const LightSource *light = scene->SampleAllLights(rndGen->floatValue(), &lightPickPdf);

		// Initialize the light path
		float lightEmitPdf;
		Ray nextEventRay;
		Spectrum lightPathFlux = light->Emit(scene,
			rndGen->floatValue(), rndGen->floatValue(), rndGen->floatValue(), rndGen->floatValue(),
			&nextEventRay.o, &nextEventRay.d, &lightEmitPdf);
		if (lightPathFlux.Black())
			continue;
		lightPathFlux /= lightEmitPdf * lightPickPdf;
		assert (!lightPathFlux.IsNaN() && !lightPathFlux.IsInf());

		// Sample a point on the camera lens
		Point lensPoint;
		if (!scene->camera->SampleLens(rndGen->floatValue(), rndGen->floatValue(),
				&lensPoint))
			continue;

		//----------------------------------------------------------------------
		// I don't try to connect the light vertex directly with the eye
		// because InfiniteLight::Emit() returns a point on the scene bounding
		// sphere. Instead, I trace a ray from the camera like in BiDir.
		// This is also a good why to test the Film Per-Pixel-Normalization and
		// the Per-Screen-Normalization Buffers used by BiDir.
		//----------------------------------------------------------------------

		{
			Ray eyeRay;
			const float screenX = min(rndGen->floatValue() * filmWidth, (float)(filmWidth - 1));
			const float screenY = min(rndGen->floatValue() * filmHeight, (float)(filmHeight - 1));
			camera->GenerateRay(screenX, screenY, &eyeRay,
				rndGen->floatValue(), rndGen->floatValue());

			Spectrum radiance, connectionThroughput;
			RayHit eyeRayHit;
			BSDF bsdf;
			const bool somethingWasHit = scene->Intersect(
				false, true, rndGen->floatValue(), &eyeRay, &eyeRayHit, &bsdf, &connectionThroughput);
			if (!somethingWasHit) {
				// Nothing was hit, check infinite lights (including sun)
				radiance += scene->GetEnvLightsRadiance(-eyeRay.d, Point());
			} else {
				// Something was hit, check if it is a light source
				if (bsdf.IsLightSource())
					radiance += bsdf.GetEmittedRadiance(scene);
			}
			radiance *= connectionThroughput;

			// Add a sample even if it is black in order to avoid aliasing problems
			// between sampled pixel and not sampled one (in PER_PIXEL_NORMALIZED buffer)
			film->AddSampleCount(PER_PIXEL_NORMALIZED, 1.0);
			film->SplatFiltered(PER_PIXEL_NORMALIZED, screenX, screenY, radiance);
			film->SplatFilteredAlpha(screenX, screenY, somethingWasHit ? 1.f : 0.f);
		}

		//----------------------------------------------------------------------
		// Trace the light path
		//----------------------------------------------------------------------

		int depth = 1;
		while (depth <= renderEngine->maxPathDepth) {
			RayHit nextEventRayHit;
			BSDF bsdf;
			Spectrum connectionThroughput;
			if (scene->Intersect(true, true, rndGen->floatValue(),
					&nextEventRay, &nextEventRayHit, &bsdf, &connectionThroughput)) {
				// Something was hit
				
				lightPathFlux *= connectionThroughput;

				// Check if it is a light source
				if (bsdf.IsLightSource()) {
					// SLG light sources are like black bodies
					break;
				}

				//--------------------------------------------------------------
				// Try to connect the light path vertex with the eye
				//--------------------------------------------------------------

				renderEngine->ConnectToEye(film, rndGen->floatValue(),
						bsdf, lensPoint, lightPathFlux);

				if (depth >= renderEngine->maxPathDepth)
					break;
				
				//--------------------------------------------------------------
				// Build the next vertex path ray
				//--------------------------------------------------------------

				float bsdfPdf;
				Vector sampledDir;
				BSDFEvent event;
				float cosSampleDir;
				const Spectrum bsdfSample = bsdf.Sample(&sampledDir,
						rndGen->floatValue(), rndGen->floatValue(), rndGen->floatValue(),
						&bsdfPdf, &cosSampleDir, &event);
				if ((bsdfPdf <= 0.f) || bsdfSample.Black())
					break;

				if (depth >= renderEngine->rrDepth) {
					// Russian Roulette
					const float prob = Max(bsdfSample.Filter(), renderEngine->rrImportanceCap);
					if (prob > rndGen->floatValue())
						bsdfPdf *= prob;
					else
						break;
				}

				lightPathFlux *= bsdfSample * (cosSampleDir / bsdfPdf);
				assert (!lightPathFlux.IsNaN() && !lightPathFlux.IsInf());

				nextEventRay = Ray(bsdf.hitPoint, sampledDir);
				++depth;
			} else {
				// Ray lost in space...
				break;
			}
		}
	}

	//SLG_LOG("[LightCPURenderThread::" << renderThread->threadIndex << "] Rendering thread halted");
}
