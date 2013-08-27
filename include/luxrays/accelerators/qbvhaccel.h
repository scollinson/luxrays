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

#ifndef _LUXRAYS_QBVHACCEL_H
#define	_LUXRAYS_QBVHACCEL_H

#include <string.h>
#include <xmmintrin.h>
#include <boost/cstdint.hpp>

#include "luxrays/luxrays.h"
#include "luxrays/core/accelerator.h"

using boost::int32_t;

namespace luxrays {

#define NODE_WIDTH 4

// This code is based on Flexray by Anthony Pajot (anthony.pajot@alumni.enseeiht.fr)

/**
   QBVH accelerator, using the EGSR08 paper as base.
   need SSE !
*/

/**
   the number of bins for construction
*/
#define NB_BINS 8

/**
   The QBVH node structure, 128 bytes long (perfect for cache)
*/
class QBVHNode {
public:
	// The constant used to represent empty leaves. there would have been
	// a conflict with a normal leaf if there were 16 quads,
	// starting at 2^27 in the quads array... very improbable.
	// using MININT (0x80000000) can produce conflict when initializing a
	// QBVH with less than 4 vertices at the beginning :
	// the number of quads - 1 would give 0, and it would start at 0
	// in the quads array
	static const int32_t emptyLeafNode = 0xffffffff;

	/**
	   The 4 bounding boxes, in SoA form, for direct SIMD use
	   (one __m128 for each coordinate)
	*/
	int32_t bboxes[NODE_WIDTH][2][3];

	/**
	   The 4 children. If a child is a leaf, its index will be negative,
	   the 4 next bits will code the number of primitives in the leaf
	   (more exactly, nbPrimitives = 4 * (p + 1), where p is the integer
	   interpretation of the 4 bits), and the 27 remaining bits the index
	   of the first quad of the node
	*/
	int32_t children[NODE_WIDTH];

	/**
	   Base constructor, init correct bounding boxes and a "root" node
	   (parentNodeIndex == -1)
	*/
	inline QBVHNode() {
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < NODE_WIDTH; j++) {
				bboxes[j][0][i] = INFINITY;
				bboxes[j][1][i] = -INFINITY;
			}
		}

		// All children are empty leaves by default
		for (int i = 0; i < NODE_WIDTH; ++i)
			children[i] = emptyLeafNode;
	}

	/**
	   Indicate whether the ith child is a leaf.
	   @param i
	   @return
	*/
	inline bool ChildIsLeaf(int i) const {
		return (children[i] < 0);
	}

	/**
	   Same thing, directly from the index.
	   @param index
	*/
	inline static bool IsLeaf(int32_t index) {
		return (index < 0);
	}

	/**
	   Indicates whether the ith child is an empty leaf.
	   @param i
	*/
	inline bool LeafIsEmpty(int i) const {
		return (children[i] == emptyLeafNode);
	}

	/**
	   Same thing, directly from the index.
	   @param index
	*/
	inline static bool IsEmpty(int32_t index) {
		return (index == emptyLeafNode);
	}

	/**
	   Indicate the number of quads in the ith child, which must be
	   a leaf.
	   @param i
	   @return
	*/
	inline u_int NbQuadsInLeaf(int i) const {
		return static_cast<u_int>((children[i] >> 27) & 0xf) + 1;
	}

	/**
	   Return the number of group of 4 primitives, directly from the index.
	   @param index
	*/
	inline static u_int NbQuadPrimitives(int32_t index) {
		return static_cast<u_int>((index >> 27) & 0xf) + 1;
	}

	/**
	   Indicate the number of primitives in the ith child, which must be
	   a leaf.
	   @param i
	   @return
	*/
	inline u_int NbPrimitivesInLeaf(int i) const {
		return NbQuadsInLeaf(i) * 4;
	}

	/**
	   Indicate the index in the quads array of the first quad contained
	   by the the ith child, which must be a leaf.
	   @param i
	   @return
	*/
	inline u_int FirstQuadIndexForLeaf(int i) const {
		return children[i] & 0x07ffffff;
	}

	/**
	   Same thing, directly from the index.
	   @param index
	*/
	inline static u_int FirstQuadIndex(int32_t index) {
		return index & 0x07ffffff;
	}

	/**
	   Initialize the ith child as a leaf
	   @param i
 	   @param nbQuads
	   @param firstQuadIndex
	*/
	inline void InitializeLeaf(int i, u_int nbQuads, u_int firstQuadIndex) {
		// Take care to make a valid initialisation of the leaf.
		if (nbQuads == 0) {
			children[i] = emptyLeafNode;
		} else {
			// Put the negative sign in a plateform independent way
			children[i] = 0x80000000;//-1L & ~(-1L >> 1L);

			children[i] |=  ((static_cast<int32_t>(nbQuads) - 1) & 0xf) << 27;

			children[i] |= static_cast<int32_t>(firstQuadIndex) & 0x07ffffff;
		}
	}

	/**
	   Set the bounding box for the ith child.
	   @param i
	   @param bbox
	*/
	inline void SetBBox(int i, const BBox &bbox) {
		for (int axis = 0; axis < 3; ++axis) {
			bboxes[i][0][axis] = bbox.pMin[axis];
			bboxes[i][1][axis] = bbox.pMax[axis];
		}
	}


	/**
	   Intersect a ray described by sse variables with the 4 bounding boxes
	   of the node.
	   (the visit array)
	*/
	inline int32_t BBoxIntersect(const Ray &ray, const float invDir[3],
		const int sign[3]) const {
		int32_t mask = 0;
		for (u_int i = 0; i < NODE_WIDTH; i++) {
			if (BBox::IntersectP(ray, bboxes[i][0], bboxes[i][1], NULL, NULL)) {
				mask |= 1 << i;
			}
		}
		return mask;
	}
};

/***************************************************/
class QBVHAccel : public Accelerator {
public:
	/**
	   Normal constructor.
	*/
	QBVHAccel(const Context *context, u_int mp, u_int fst, u_int sf);

	/**
	   to free the memory.
	*/
	virtual ~QBVHAccel();

	virtual AcceleratorType GetType() const { return ACCEL_QBVH; }
	virtual OpenCLKernels *NewOpenCLKernels(OpenCLIntersectionDevice *device,
		const u_int kernelCount, const u_int stackSize, const bool enableImageStorage) const;
	virtual bool CanRunOnOpenCLDevice(OpenCLIntersectionDevice *device) const;
	virtual void Init(const std::deque<const Mesh *> &meshes,
		const u_longlong totalVertexCount,
		const u_longlong totalTriangleCount);

	/**
	   Intersect a ray in world space against the
	   primitive and fills in an Intersection object.
	*/
	virtual bool Intersect(const Ray *ray, RayHit *hit) const;

	friend class MQBVHAccel;

private:
	// A special initialization method used only by MQBVHAccel
	void Init(const Mesh *m, const TriangleMeshID *preprocessedMeshIDs);

	/**
	   Build the tree that will contain the primitives indexed from start
	   to end in the primsIndexes array.
	*/
	void BuildTree(u_int start, u_int end, std::vector<u_int> &meshIndexes, std::vector<u_int> &triangleIndexes,
		std::vector<std::vector<BBox> > &primsBboxes, std::vector<std::vector<Point> > &primsCentroids, const BBox &nodeBbox,
		const BBox &centroidsBbox, int32_t parentIndex,
		int32_t childIndex, int depth);

	/**
	   Create a leaf using the traditional QBVH layout
	*/
	void CreateTempLeaf(int32_t parentIndex, int32_t childIndex,
		u_int start, u_int end, const BBox &nodeBbox);

	/**
	   Create an intermediate node
	*/
	inline int32_t CreateIntermediateNode(int32_t parentIndex,
		int32_t childIndex, const BBox &nodeBbox) {
		int32_t index = nNodes++; // increment after assignment
		if (nNodes >= maxNodes) {
			QBVHNode *newNodes = AllocAligned<QBVHNode>(2 * maxNodes);
			memcpy(newNodes, nodes, sizeof(QBVHNode) * maxNodes);
			for (u_int i = 0; i < maxNodes; ++i)
				newNodes[maxNodes + i] = QBVHNode();
			FreeAligned(nodes);
			nodes = newNodes;
			maxNodes *= 2;
		}

		if (parentIndex >= 0) {
			nodes[parentIndex].children[childIndex] = index;
			nodes[parentIndex].SetBBox(childIndex, nodeBbox);
		}
		return index;
	}

	/**
	   switch a node and its subnodes from the
	   traditional form of QBVH to the pre-swizzled one.
	*/
	void PreSwizzle(int32_t nodeIndex, std::vector<u_int> &meshIndexes, std::vector<u_int> &triangleIndexes);

	/**
	   Create a leaf using the pre-swizzled layout,
	   using the informations stored in the node that
	   are organized following the traditional layout
	*/
	void CreateSwizzledLeaf(int32_t parentIndex, int32_t childIndex,
		std::vector<u_int> &meshIndexes, std::vector<u_int> &triangleIndexes);

	/**
	   the actual number of quads
	*/
	u_int nQuads;

	/**
	   The primitive associated with each triangle. indexed by the number of quad
	   and the number of triangle in the quad (thus, there might be holes).
	   no need to be a tesselated primitive, the intersection
	   test will be redone for the nearest triangle found, to
	   fill the Intersection structure.
	*/
	QuadTriangle *prims;

	/**
	   The nodes of the QBVH.
	*/
	QBVHNode *nodes;

	/**
	   The number of nodes really used.
	*/
	u_int nNodes, maxNodes;

	/**
	   The world bounding box of the QBVH.
	*/
	BBox worldBound;

	/**
	   The number of primitives in the node that makes switch
	   to full sweep for binning
	*/
	u_int fullSweepThreshold;

	/**
	   The skip factor for binning
	*/
	u_int skipFactor;

	/**
	   The maximum number of primitives per leaf
	*/
	u_int maxPrimsPerLeaf;

	const Context *ctx;
	std::deque<const Mesh *> meshes;

	int maxDepth;

	bool initialized;
};

}

#endif	/* _LUXRAYS_QBVHACCEL_H */
