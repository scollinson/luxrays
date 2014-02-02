#include "slg/engines/pathfpga/pathfpga.h"
#include "slg/core/mc.h"

using namespace std;
using namespace luxrays;
using namespace slg;

// TODO: use only brute force to sample infinitelight

//------------------------------------------------------------------------------
// PathFPGA RenderThread
//------------------------------------------------------------------------------

PathFPGARenderThread::PathFPGARenderThread(PathFPGARenderEngine *engine,
		const u_int index, IntersectionDevice *device) :
		FPGARenderThread(engine, index, device, true, false) {
}

void PathFPGARenderThread::DirectLightSampling(
		const float u0, const float u1, const float u2,
		const float u3, const float u4,
		const Spectrum &pathThrouput, const BSDF &bsdf,
		const int depth, Spectrum *radiance) {
	PathFPGARenderEngine *engine = (PathFPGARenderEngine *)renderEngine;
	Scene *scene = engine->renderConfig->scene;
	
	if (!bsdf.IsDelta()) {
		// Pick a light source to sample
		float lightPickPdf;
		const LightSource *light = scene->SampleAllLights(u0, &lightPickPdf);

		Vector lightRayDir;
		float distance, directPdfW;
		Spectrum lightRadiance = light->Illuminate(*scene, bsdf.hitPoint.p,
				u1, u2, u3, &lightRayDir, &distance, &directPdfW);

		if (!lightRadiance.Black()) {
			BSDFEvent event;
			float bsdfPdfW;
			Spectrum bsdfEval = bsdf.Evaluate(lightRayDir, &event, &bsdfPdfW);

			if (!bsdfEval.Black()) {
				const float epsilon = Max(MachineEpsilon::E(bsdf.hitPoint.p), MachineEpsilon::E(distance));
				Ray shadowRay(bsdf.hitPoint.p, lightRayDir,
						epsilon,
						distance - epsilon);
				RayHit shadowRayHit;
				BSDF shadowBsdf;
				Spectrum connectionThroughput;
				// Check if the light source is visible
				if (!scene->Intersect(device, false, u4, &shadowRay,
						&shadowRayHit, &shadowBsdf, &connectionThroughput)) {
					const float cosThetaToLight = AbsDot(lightRayDir, bsdf.hitPoint.shadeN);
					const float directLightSamplingPdfW = directPdfW * lightPickPdf;
					const float factor = cosThetaToLight / directLightSamplingPdfW;

					if (depth >= engine->rrDepth) {
						// Russian Roulette
						bsdfPdfW *= RenderEngine::RussianRouletteProb(bsdfEval, engine->rrImportanceCap);
					}

					// MIS between direct light sampling and BSDF sampling
					const float weight = PowerHeuristic(directLightSamplingPdfW, bsdfPdfW);

					*radiance += (weight * factor) * pathThrouput * connectionThroughput * lightRadiance * bsdfEval;
				}
			}
		}
	}
}

void PathFPGARenderThread::DirectHitFiniteLight(
		const bool lastSpecular, const Spectrum &pathThrouput,
		const float distance, const BSDF &bsdf, const float lastPdfW,
		Spectrum *radiance) {
	PathFPGARenderEngine *engine = (PathFPGARenderEngine *)renderEngine;
	Scene *scene = engine->renderConfig->scene;

	float directPdfA;
	const Spectrum emittedRadiance = bsdf.GetEmittedRadiance(&directPdfA);

	if (!emittedRadiance.Black()) {
		float weight;
		if (!lastSpecular) {
			const float lightPickProb = scene->PickLightPdf();
			const float directPdfW = PdfAtoW(directPdfA, distance,
				AbsDot(bsdf.hitPoint.fixedDir, bsdf.hitPoint.shadeN));

			// MIS between BSDF sampling and direct light sampling
			weight = PowerHeuristic(lastPdfW, directPdfW * lightPickProb);
		} else
			weight = 1.f;

		*radiance +=  pathThrouput * weight * emittedRadiance;
	}
}

void PathFPGARenderThread::DirectHitInfiniteLight(
		const bool lastSpecular, const Spectrum &pathThrouput,
		const Vector &eyeDir, const float lastPdfW, Spectrum *radiance) {
	PathFPGARenderEngine *engine = (PathFPGARenderEngine *)renderEngine;
	Scene *scene = engine->renderConfig->scene;

	// Infinite light
	float directPdfW;
	if (scene->envLight) {
		const Spectrum envRadiance = scene->envLight->GetRadiance(*scene, -eyeDir, &directPdfW);
		if (!envRadiance.Black()) {
			if(!lastSpecular) {
				// MIS between BSDF sampling and direct light sampling
				*radiance += pathThrouput * PowerHeuristic(lastPdfW, directPdfW) * envRadiance;
			} else
				*radiance += pathThrouput * envRadiance;
		}
	}

	// Sun light
	if (scene->sunLight) {
		const Spectrum sunRadiance = scene->sunLight->GetRadiance(*scene, -eyeDir, &directPdfW);
		if (!sunRadiance.Black()) {
			if(!lastSpecular) {
				// MIS between BSDF sampling and direct light sampling
				*radiance += pathThrouput * PowerHeuristic(lastPdfW, directPdfW) * sunRadiance;
			} else
				*radiance += pathThrouput * sunRadiance;
		}
	}
}

void PathFPGARenderThread::RenderFunc() {
	//SLG_LOG("[PathFPGARenderEngine::" << threadIndex << "] Rendering thread started");

	//--------------------------------------------------------------------------
	// Initialization
	//--------------------------------------------------------------------------

	PathFPGARenderEngine *engine = (PathFPGARenderEngine *)renderEngine;
	RandomGenerator *rndGen = new RandomGenerator(engine->seedBase + threadIndex);
	Scene *scene = engine->renderConfig->scene;
	PerspectiveCamera *camera = scene->camera;
	Film * film = threadFilm;
	const unsigned int filmWidth = film->GetWidth();
	const unsigned int filmHeight = film->GetHeight();

	// Setup the sampler
	double metropolisSharedTotalLuminance, metropolisSharedSampleCount;
	Sampler *sampler = engine->renderConfig->AllocSampler(rndGen, film,
			&metropolisSharedTotalLuminance, &metropolisSharedSampleCount);
	const unsigned int sampleBootSize = 4;
	const unsigned int sampleStepSize = 9;
	const unsigned int sampleSize = 
		sampleBootSize + // To generate eye ray
		engine->maxPathDepth * sampleStepSize; // For each path vertex
	sampler->RequestSamples(sampleSize);

	//--------------------------------------------------------------------------
	// Trace paths
	//--------------------------------------------------------------------------

	vector<SampleResult> sampleResults(1);
	sampleResults[0].type = PER_PIXEL_NORMALIZED;
	while (!boost::this_thread::interruption_requested()) {
		float alpha = 1.f;

		Ray eyeRay;
		const float screenX = Min(sampler->GetSample(0) * filmWidth, (float)(filmWidth - 1));
		const float screenY = Min(sampler->GetSample(1) * filmHeight, (float)(filmHeight - 1));
		camera->GenerateRay(screenX, screenY, &eyeRay,
			sampler->GetSample(2), sampler->GetSample(3));

		int depth = 1;
		bool lastSpecular = true;
		float lastPdfW = 1.f;
		Spectrum radiance;
		Spectrum pathThrouput(1.f, 1.f, 1.f);
		BSDF bsdf;
		while (depth <= engine->maxPathDepth) {
			const unsigned int sampleOffset = sampleBootSize + (depth - 1) * sampleStepSize;

			RayHit eyeRayHit;
			Spectrum connectionThroughput;
			if (!scene->Intersect(device, false, sampler->GetSample(sampleOffset),
					&eyeRay, &eyeRayHit, &bsdf, &connectionThroughput)) {
				// Nothing was hit, look for infinitelight
				DirectHitInfiniteLight(lastSpecular, pathThrouput * connectionThroughput, eyeRay.d,
						lastPdfW, &radiance);

				if (depth == 1)
					alpha = 0.f;
				break;
			}
			pathThrouput *= connectionThroughput;

			// Something was hit

			// Check if it is a light source
			if (bsdf.IsLightSource()) {
				DirectHitFiniteLight(lastSpecular, pathThrouput,
						eyeRayHit.t, bsdf, lastPdfW, &radiance);
			}

			// Note: pass-through check is done inside SceneIntersect()

			//------------------------------------------------------------------
			// Direct light sampling
			//------------------------------------------------------------------

			DirectLightSampling(sampler->GetSample(sampleOffset + 1),
					sampler->GetSample(sampleOffset + 2),
					sampler->GetSample(sampleOffset + 3),
					sampler->GetSample(sampleOffset + 4),
					sampler->GetSample(sampleOffset + 5),
					pathThrouput, bsdf, depth, &radiance);

			//------------------------------------------------------------------
			// Build the next vertex path ray
			//------------------------------------------------------------------

			Vector sampledDir;
			BSDFEvent event;
			float cosSampledDir;
			const Spectrum bsdfSample = bsdf.Sample(&sampledDir,
					sampler->GetSample(sampleOffset + 6),
					sampler->GetSample(sampleOffset + 7),
					&lastPdfW, &cosSampledDir, &event);
			if (bsdfSample.Black())
				break;

			lastSpecular = ((event & SPECULAR) != 0);

			if ((depth >= engine->rrDepth) && !lastSpecular) {
				// Russian Roulette
				const float prob = RenderEngine::RussianRouletteProb(bsdfSample, engine->rrImportanceCap);
				if (sampler->GetSample(sampleOffset + 8) < prob)
					lastPdfW *= prob;
				else
					break;
			}

			pathThrouput *= bsdfSample * (cosSampledDir / lastPdfW);
			assert (!pathThrouput.IsNaN() && !pathThrouput.IsInf());

			eyeRay = Ray(bsdf.hitPoint.p, sampledDir);
			++depth;
		}

		assert (!radiance.IsNaN() && !radiance.IsInf());

		sampleResults[0].screenX = screenX;
		sampleResults[0].screenY = screenY;
		sampleResults[0].radiance = radiance;
		sampleResults[0].alpha = alpha;
		sampler->NextSample(sampleResults);

#ifdef WIN32
		// Work around Windows bad scheduling
		renderThread->yield();
#endif
	}

	delete sampler;
	delete rndGen;

	//SLG_LOG("[PathFPGARenderEngine::" << threadIndex << "] Rendering thread halted");
}
