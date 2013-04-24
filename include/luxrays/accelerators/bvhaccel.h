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

#ifndef _LUXRAYS_BVHACCEL_H
#define	_LUXRAYS_BVHACCEL_H

#include <vector>

#include "luxrays/luxrays.h"
#include "luxrays/core/accelerator.h"

namespace luxrays {

struct BVHAccelTreeNode {
	BBox bbox;
	u_int primitive;
	BVHAccelTreeNode *leftChild;
	BVHAccelTreeNode *rightSibling;
};

struct BVHAccelArrayNode {
	BBox bbox;
	u_int primitive;
	u_int skipIndex;
};

// BVHAccel Declarations
class BVHAccel : public Accelerator {
public:
	// BVHAccel Public Methods
	BVHAccel(const Context *context,
			const u_int treetype, const int csamples, const int icost,
			const int tcost, const float ebonus);
	virtual ~BVHAccel();

	virtual AcceleratorType GetType() const { return ACCEL_BVH; }
	virtual OpenCLKernels *NewOpenCLKernels(OpenCLIntersectionDevice *device,
		const u_int kernelCount, const u_int stackSize, const bool disableImageStorage) const;
	virtual void Init(const std::deque<const Mesh *> &meshes,
		const u_int totalVertexCount,
		const u_int totalTriangleCount);

	virtual const TriangleMeshID GetMeshID(const u_int index) const {
		return meshIDs[index];
	}
	virtual const TriangleMeshID *GetMeshIDTable() const {
		return meshIDs;
	}
	virtual const TriangleID GetMeshTriangleID(const u_int index) const {
		return meshTriangleIDs[index];
	}
	virtual const TriangleID *GetMeshTriangleIDTable() const {
		return meshTriangleIDs;
	}

	virtual bool Intersect(const Ray *ray, RayHit *hit) const;

	friend class MBVHAccel;

private:
	typedef struct {
		u_int treeType;
		int costSamples, isectCost, traversalCost;
		float emptyBonus;
	} BVHParams;

	// BVHAccel Private Methods
	static BVHAccelTreeNode *BuildHierarchy(u_int *nNodes, const BVHParams &params,
		std::vector<BVHAccelTreeNode *> &list,
		u_int begin, u_int end, u_int axis);
	static void FreeHierarchy(BVHAccelTreeNode *node);
	static void FindBestSplit(const BVHParams &params,
		std::vector<BVHAccelTreeNode *> &list,
		u_int begin, u_int end, float *splitValue,
		u_int *bestAxis);

	static u_int BuildArray(BVHAccelTreeNode *node, u_int offset, BVHAccelArrayNode *bvhTree);

	// A special initialization method used only by MBVHAccel
	void Init(const Mesh *m);

	BVHParams params;

	u_int nNodes;
	BVHAccelArrayNode *bvhTree;

	const Context *ctx;
	TriangleMesh *preprocessedMesh;
	const Mesh *mesh;
	TriangleMeshID *meshIDs;
	TriangleID *meshTriangleIDs;

	bool initialized;
};

}

#endif	/* _LUXRAYS_BVHACCEL_H */
