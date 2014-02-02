#include "slg/engines/pathfpga/pathfpga.h"

using namespace std;
using namespace luxrays;
using namespace slg;

//------------------------------------------------------------------------------
// PathFPGARenderEngine
//------------------------------------------------------------------------------

PathFPGARenderEngine::PathFPGARenderEngine(RenderConfig *rcfg, Film *flm, boost::mutex *flmMutex) :
		FPGARenderEngine(rcfg, flm, flmMutex) {
	film->SetPerPixelNormalizedBufferFlag(true);
	film->SetPerScreenNormalizedBufferFlag(false);
	film->SetOverlappedScreenBufferUpdateFlag(true);
	film->Init();
}

void PathFPGARenderEngine::StartLockLess() {
	const Properties &cfg = renderConfig->cfg;

	//--------------------------------------------------------------------------
	// Rendering parameters
	//--------------------------------------------------------------------------

	maxPathDepth = cfg.GetInt("path.maxdepth", 5);
	rrDepth = cfg.GetInt("path.russianroulette.depth", 3);
	rrImportanceCap = cfg.GetFloat("path.russianroulette.cap", .5f);

	FPGARenderEngine::StartLockLess();
}