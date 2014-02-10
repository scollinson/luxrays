/***************************************************************************
 *   Copyright (C) 1998-2013 by authors (see AUTHORS.txt)                  *
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

#ifndef _SLG_PATHFPGA_H
#define	_SLG_PATHFPGA_H

#include "luxrays/core/randomgen.h"
#include "slg/slg.h"
#include "slg/renderengine.h"
#include "slg/sampler/sampler.h"
#include "slg/film/film.h"
#include "slg/sdl/bsdf.h"

namespace slg {

//------------------------------------------------------------------------------
// Path tracing FPGA render engine
//------------------------------------------------------------------------------

class PathFPGARenderThread;
class PathFPGARenderEngine;

class PathFPGAState : public FPGARenderState {
public:
	PathFPGAState(PathFPGARenderThread *renderThread, Film *film, luxrays::RandomGenerator *rndGen);
	virtual ~PathFPGAState() { }

	virtual void GenerateRays(FPGARenderThread *renderThread);
	virtual double CollectResults(FPGARenderThread *renderThread);

private:
	void Init(const PathFPGARenderThread *thread);
	void DirectHitInfiniteLight(const slg::Scene *scene, const luxrays::Vector &eyeDir);
	void DirectHitFiniteLight(const Scene *scene, const float distance, const BSDF &bsdf);
	void DirectLightSampling(const PathFPGARenderThread *renderThread,
		const float u0, const float u1,
		const float u2, const float u3,
		const BSDF &bsdf);
	bool FinalizeRay(const PathFPGARenderThread *renderThread, const luxrays::Ray *ray,
		const luxrays::RayHit *rayHit, BSDF *bsdf, const float u0,
		luxrays::Spectrum *radiance);
	void SplatSample(const PathFPGARenderThread *renderThread);

	int depth;
	float lastPdfW;
	luxrays::Spectrum throuput;

	luxrays::Ray nextPathVertexRay, directLightRay;
	luxrays::Spectrum directLightRadiance;

	vector<slg::SampleResult> sampleResults;

	bool lastSpecular;
};

class PathFPGARenderThread : public FPGARenderThread {
public:
	PathFPGARenderThread(PathFPGARenderEngine *engine, const u_int index,
			luxrays::IntersectionDevice *device);

	friend class PathFPGAState;
	friend class PathFPGARenderEngine;

private:
	FPGARenderState *AllocRenderState(luxrays::RandomGenerator *rndGen) {
		return new PathFPGAState(this, threadFilm, rndGen);
	}
	boost::thread *AllocRenderThread() {
		return new boost::thread(&PathFPGARenderThread::RenderFunc, this);
	}
};

class PathFPGARenderEngine : public FPGARenderEngine {
public:
	PathFPGARenderEngine(RenderConfig *cfg, Film *flm, boost::mutex *flmMutex);

	RenderEngineType GetEngineType() const { return PATHFPGA; }

	// Signed because of the delta parameter
	int maxPathDepth;

	int rrDepth;
	float rrImportanceCap;

	friend class PathFPGAState;
	friend class PathFPGARenderThread;

protected:
	virtual void StartLockLess();

private:
	FPGARenderThread *NewRenderThread(const u_int index, luxrays::IntersectionDevice *device) {
		return new PathFPGARenderThread(this, index, device);
	}
};

}

#endif	/* _SLG_PATHFPGA_H */
