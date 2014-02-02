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

class PathFPGARenderEngine;

class PathFPGARenderThread : public FPGARenderThread {
public:
	PathFPGARenderThread(PathFPGARenderEngine *engine, const u_int index,
			luxrays::IntersectionDevice *device);

	friend class PathFPGARenderEngine;

private:
	boost::thread *AllocRenderThread() { return new boost::thread(&PathFPGARenderThread::RenderFunc, this); }

	void RenderFunc();

	void DirectLightSampling(const float u0, const float u1,
			const float u2, const float u3, const float u4,
			const luxrays::Spectrum &pathThrouput, const BSDF &bsdf, const int depth,
			luxrays::Spectrum *radiance);

	void DirectHitFiniteLight(const bool lastSpecular,
			const luxrays::Spectrum &pathThrouput, const float distance, const BSDF &bsdf,
			const float lastPdfW, luxrays::Spectrum *radiance);

	void DirectHitInfiniteLight(const bool lastSpecular, const luxrays::Spectrum &pathThrouput,
			const luxrays::Vector &eyeDir, const float lastPdfW, luxrays::Spectrum *radiance);
};

class PathFPGARenderEngine : public FPGARenderEngine {
public:
	PathFPGARenderEngine(RenderConfig *cfg, Film *flm, boost::mutex *flmMutex);

	RenderEngineType GetEngineType() const { return PATHFPGA; }

	virtual bool IsHorizontalStereoSupported() const {
		return true;
	}

	// Signed because of the delta parameter
	int maxPathDepth;

	int rrDepth;
	float rrImportanceCap;

	friend class PathFPGARenderThread;

protected:
	virtual void StartLockLess();

private:
	FPGARenderThread *NewRenderThread(const u_int index,
			luxrays::IntersectionDevice *device) {
		return new PathFPGARenderThread(this, index, device);
	}
};

}

#endif