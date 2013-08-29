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

#ifndef _LUXRAYS_NBVHACCEL_H
#define	_LUXRAYS_NBVHACCEL_H

#include <string.h>
#include <xmmintrin.h>
#include <boost/cstdint.hpp>

#include "luxrays/luxrays.h"
#include "luxrays/core/accelerator.h"

using boost::int32_t;

namespace luxrays {

#define NODE_WIDTH_LOG2 3
#define NODE_WIDTH 8


// This code is based on Flexray by Anthony Pajot (anthony.pajot@alumni.enseeiht.fr)

/**
   NBVH accelerator, using the EGSR08 paper as base.
   need SSE !
*/

struct BVH {
	BBox box;
	int32_t child;
	int32_t pad;
};

class NTriangle : public Aligned16 {
public:

	NTriangle() { };

	NTriangle(const std::deque<const Mesh *> &meshes, const unsigned int mi,
	          const unsigned int ti) {

		meshIndex = mi;
		triangleIndex = ti;

		const Mesh *mesh = meshes[meshIndex];
		const Triangle *t = &(mesh->GetTriangles()[triangleIndex]);

		const Point p0 = mesh->GetVertex(t->v[0]);
		const Point p1 = mesh->GetVertex(t->v[1]);
		const Point p2 = mesh->GetVertex(t->v[2]);

		o.x = p0.x;
		o.y = p0.y;
		o.z = p0.z;

		e1.x = p1.x - p0.x;
		e1.y = p1.y - p0.y;
		e1.z = p1.z - p0.z;

		e2.x = p2.x - p0.x;
		e2.y = p2.y - p0.y;
		e2.z = p2.z - p0.z;
	}

	~NTriangle() {
	}

	bool Intersect(const Ray &ray, RayHit *rayHit) const {
		const Vector s1 = Cross(ray.d, e2);

		const float divisor = Dot(s1, e1);
		if (divisor == 0.f)
			return false;

		const float invDivisor = 1.f / divisor;

		// Compute first barycentric coordinate
		const Vector d = ray.o - o;
		const float b1 = Dot(d, s1) * invDivisor;
		if (b1 < 0.f)
			return false;

		// Compute second barycentric coordinate
		const Vector s2 = Cross(d, e1);
		const float b2 = Dot(ray.d, s2) * invDivisor;
		if (b2 < 0.f)
			return false;

		const float b0 = 1.f - b1 - b2;
		if (b0 < 0.f)
			return false;

		// Compute _t_ to intersection point
		const float t = Dot(e2, s2) * invDivisor;
		if (t < ray.mint || t > ray.maxt)
			return false;

		ray.maxt = t;
		rayHit->t = ray.maxt;
		rayHit->b1 = b1;
		rayHit->b2 = b2;
		rayHit->meshIndex = meshIndex;
		rayHit->triangleIndex = triangleIndex;

		return true;
	}

	Point o;
	Vector e1, e2;
	unsigned int meshIndex, triangleIndex, pad;
};

/**
   the number of bins for construction
*/
#define NB_BINS 8

/**
   The NBVH node structure, 128 bytes long (perfect for cache)
*/
class NBVHNode {
public:
	// The constant used to represent empty leaves. there would have been
	// a conflict with a normal leaf if there were 16 quads,
	// starting at 2^27 in the quads array... very improbable.
	// using MININT (0x80000000) can produce conflict when initializing a
	// NBVH with less than 4 vertices at the beginning :
	// the number of quads - 1 would give 0, and it would start at 0
	// in the quads array
	static const int32_t emptyLeafNode = 0xffffffff;

	/**
	   The 4 bounding boxes, in SoA form, for direct SIMD use
	   (one __m128 for each coordinate)
	*/
	BVH bvhs[NODE_WIDTH];

	/**
	   The 4 children. If a child is a leaf, its index will be negative,
	   the 4 next bits will code the number of primitives in the leaf
	   (more exactly, nbPrimitives = 4 * (p + 1), where p is the integer
	   interpretation of the 4 bits), and the 27 remaining bits the index
	   of the first quad of the node
	*/

	/**
	   Base constructor, init correct bounding boxes and a "root" node
	   (parentNodeIndex == -1)
	*/
	inline NBVHNode() {
		for (int i = 0; i < NODE_WIDTH; i++) {
			bvhs[i].box = BBox();
			bvhs[i].child = emptyLeafNode;
		}
	}

	/**
	   Indicate whether the ith child is a leaf.
	   @param i
	   @return
	*/
	inline bool ChildIsLeaf(int i) const {
		return (bvhs[i].child < 0);
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
		return (bvhs[i].child == emptyLeafNode);
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
		return static_cast<u_int>((bvhs[i].child >> 27) & 0xf) + 1;
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
		return bvhs[i].child & 0x07ffffff;
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
			bvhs[i].child = emptyLeafNode;
		} else {
			// Put the negative sign in a plateform independent way
			bvhs[i].child = 0x80000000;//-1L & ~(-1L >> 1L);

			bvhs[i].child |=  ((static_cast<int32_t>(nbQuads) - 1) & 0xf) << 27;

			bvhs[i].child |= static_cast<int32_t>(firstQuadIndex) & 0x07ffffff;
		}
	}

	/**
	   Set the bounding box for the ith child.
	   @param i
	   @param bbox
	*/
	inline void SetBBox(int i, const BBox &bbox) {
		bvhs[i].box = bbox;
	}


	/**
	   Intersect a ray described by sse variables with the 4 bounding boxes
	   of the node.
	   (the visit array)
	*/
	inline int32_t BBoxIntersect(const Ray &ray) const {
		int32_t mask = 0;
		for (u_int i = 0; i < NODE_WIDTH; i++) {
			if (bvhs[i].box.IntersectP(ray)) {
				mask |= 1 << i;
			}
		}
		return mask;
	}
};

/***************************************************/
class NBVHAccel : public Accelerator {
public:
	/**
	   Normal constructor.
	*/
	NBVHAccel(const Context *context, u_int mp, u_int fst, u_int sf);

	/**
	   to free the memory.
	*/
	virtual ~NBVHAccel();

	virtual AcceleratorType GetType() const { return ACCEL_NBVH; }
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

	friend class MNBVHAccel;

private:
	// A special initialization method used only by MNBVHAccel
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
	   Create a leaf using the traditional NBVH layout
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
			NBVHNode *newNodes = AllocAligned<NBVHNode>(2 * maxNodes);
			memcpy(newNodes, nodes, sizeof(NBVHNode) * maxNodes);
			for (u_int i = 0; i < maxNodes; ++i)
				newNodes[maxNodes + i] = NBVHNode();
			FreeAligned(nodes);
			nodes = newNodes;
			maxNodes *= 2;
		}

		if (parentIndex >= 0) {
			nodes[parentIndex].bvhs[childIndex].child = index;
			nodes[parentIndex].SetBBox(childIndex, nodeBbox);
		}
		return index;
	}

	/**
	   switch a node and its subnodes from the
	   traditional form of NBVH to the pre-swizzled one.
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
	NTriangle *prims;

	/**
	   The nodes of the NBVH.
	*/
	NBVHNode *nodes;

	/**
	   The number of nodes really used.
	*/
	u_int nNodes, maxNodes;

	/**
	   The world bounding box of the NBVH.
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

#endif	/* _LUXRAYS_NBVHACCEL_H */
