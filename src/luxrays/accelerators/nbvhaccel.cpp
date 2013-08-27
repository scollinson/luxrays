/***************************************************************************
 *   Copyright (C) 2007 by Anthony Pajot
 *   anthony.pajot@etu.enseeiht.fr
 *
 * This file is part of FlexRay
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 ***************************************************************************/

#include "luxrays/accelerators/qbvhaccel.h"
#include "luxrays/core/utils.h"
#include "luxrays/core/context.h"
#include "luxrays/core/intersectiondevice.h"

namespace luxrays {

/***************************************************/

OpenCLKernels *NBVHAccel::NewOpenCLKernels(OpenCLIntersectionDevice *device, const u_int kernelCount,
		const u_int stackSize, const bool enableImageStorage) const {
	return NULL;
}

bool NBVHAccel::CanRunOnOpenCLDevice(OpenCLIntersectionDevice *device) const {
	return false;
}

NBVHAccel::NBVHAccel(const Context *context,
		u_int mp, u_int fst, u_int sf) : fullSweepThreshold(fst),
		skipFactor(sf), maxPrimsPerLeaf(mp), ctx(context) {
	initialized = false;
	maxDepth = 0;
}

NBVHAccel::~NBVHAccel() {
	if (initialized) {
		FreeAligned(prims);
		FreeAligned(nodes);
	}
}

void NBVHAccel::Init(const std::deque<const Mesh *> &ms, const u_longlong totalVertexCount,
		const u_longlong totalTriangleCount) {
	assert (!initialized);

	meshes = ms;

	// Temporary data for building
	std::vector<u_int> meshIndexes(totalTriangleCount + 3);
	std::vector<u_int> triangleIndexes(totalTriangleCount + 3); // For the case where
	// the last quad would begin at the last primitive
	// (or the second or third last primitive)

	// The number of nodes depends on the number of primitives,
	// and is bounded by 2 * nPrims - 1.
	// Even if there will normally have at least 4 primitives per leaf,
	// it is not always the case => continue to use the normal bounds.
	nNodes = 0;
	maxNodes = 1;
	for (u_int layer = ((totalTriangleCount + maxPrimsPerLeaf - 1) / maxPrimsPerLeaf + 3) / 4; layer != 1; layer = (layer + 3) / 4)
		maxNodes += layer;
	nodes = AllocAligned<NBVHNode>(maxNodes);
	for (u_int i = 0; i < maxNodes; ++i)
		nodes[i] = NBVHNode();

	// The arrays that will contain
	// - the bounding boxes for all triangles
	// - the centroids for all triangles
	std::vector<std::vector<BBox> > primsBboxes(totalTriangleCount);
	std::vector<std::vector<Point> > primsCentroids(totalTriangleCount);
	// The bouding volume of all the centroids
	BBox centroidsBbox;

	// Fill each base array
	u_int absoluteIndex = 0;
	for (u_int i = 0; i < meshes.size(); ++i) {
		const Mesh *mesh = meshes[i];
		const Triangle *p = mesh->GetTriangles();
		primsBboxes[i].resize(mesh->GetTotalTriangleCount());
		primsCentroids[i].resize(mesh->GetTotalTriangleCount());

		for (u_int j = 0; j < mesh->GetTotalTriangleCount(); ++j) {
			// This array will be reorganized during construction.
			meshIndexes[absoluteIndex] = i;
			triangleIndexes[absoluteIndex++] = j;

			// Compute the bounding box for the triangle
			primsBboxes[i][j] = Union(
					BBox(mesh->GetVertex(p[j].v[0]), mesh->GetVertex(p[j].v[1])),
					mesh->GetVertex(p[j].v[2]));
			primsBboxes[i][j].Expand(MachineEpsilon::E(primsBboxes[i][j]));
			primsCentroids[i][j] = (primsBboxes[i][j].pMin + primsBboxes[i][j].pMax) * .5f;

			// Update the global bounding boxes
			worldBound = Union(worldBound, primsBboxes[i][j]);
			centroidsBbox = Union(centroidsBbox, primsCentroids[i][j]);
		}
	}

	// Arbitrarily take the last primitive for the last 3
	meshIndexes[totalTriangleCount] = meshIndexes[totalTriangleCount - 1];
	meshIndexes[totalTriangleCount + 1] = meshIndexes[totalTriangleCount - 1];
	meshIndexes[totalTriangleCount + 2] = meshIndexes[totalTriangleCount - 1];
	triangleIndexes[totalTriangleCount] = triangleIndexes[totalTriangleCount - 1];
	triangleIndexes[totalTriangleCount + 1] = triangleIndexes[totalTriangleCount - 1];
	triangleIndexes[totalTriangleCount + 2] = triangleIndexes[totalTriangleCount - 1];

	// Recursively build the tree
	LR_LOG(ctx, "Building NBVH, primitives: " << totalTriangleCount << ", initial nodes: " << maxNodes);

	nQuads = 0;
	BuildTree(0, totalTriangleCount, meshIndexes, triangleIndexes, primsBboxes, primsCentroids,
			worldBound, centroidsBbox, -1, 0, 0);

	prims = AllocAligned<QuadTriangle>(nQuads);
	nQuads = 0;
	PreSwizzle(0, meshIndexes, triangleIndexes);

	LR_LOG(ctx, "NBVH completed with " << nNodes << "/" << maxNodes << " nodes");
	LR_LOG(ctx, "Total NBVH memory usage: " << nNodes * sizeof(NBVHNode) / 1024 << "Kbytes");
	LR_LOG(ctx, "Total NBVH QuadTriangle count: " << nQuads);
	LR_LOG(ctx, "Max. NBVH Depth: " << maxDepth);

	initialized = true;
}

/***************************************************/

void NBVHAccel::BuildTree(u_int start, u_int end, std::vector<u_int> &meshIndexes, std::vector<u_int> &triangleIndexes,
		std::vector<std::vector<BBox> > &primsBboxes, std::vector<std::vector<Point> > &primsCentroids, const BBox &nodeBbox,
		const BBox &centroidsBbox, int32_t parentIndex, int32_t childIndex, int depth) {
	maxDepth = (depth >= maxDepth) ? depth : maxDepth; // Set depth so we know how much stack we need later.

	// Create a leaf ?
	//********
	if (depth > 64 || end - start <= maxPrimsPerLeaf) {
		if (depth > 64) {
			LR_LOG(ctx, "Maximum recursion depth reached while constructing NBVH, forcing a leaf node");
			if (end - start > 64) {
				LR_LOG(ctx, "NBVH unable to handle geometry, too many primitives in leaf");
			}
		}
		CreateTempLeaf(parentIndex, childIndex, start, end, nodeBbox);
		return;
	}

	// Number of primitives in each bin
	int bins[NB_BINS];
	// Bbox of the primitives in the bin
	BBox binsBbox[NB_BINS];

	//--------------
	// Fill in the bins, considering all the primitives when a given
	// threshold is reached, else considering only a portion of the
	// primitives for the binned-SAH process. Also compute the bins bboxes
	// for the primitives.

	for (u_int i = 0; i < NB_BINS; ++i)
		bins[i] = 0;

	u_int step = (end - start < fullSweepThreshold) ? 1 : skipFactor;

	// Choose the split axis, taking the axis of maximum extent for the
	// centroids (else weird cases can occur, where the maximum extent axis
	// for the nodeBbox is an axis of 0 extent for the centroids one.).
	const int axis = centroidsBbox.MaximumExtent();

	// Precompute values that are constant with respect to the current
	// primitive considered.
	const float k0 = centroidsBbox.pMin[axis];
	const float k1 = NB_BINS / (centroidsBbox.pMax[axis] - k0);

	// If the bbox is a point, create a leaf, hoping there are not more
	// than 64 primitives that share the same center.
	if (k1 == INFINITY) {
		if (end - start > 64)
			LR_LOG(ctx, "NBVH unable to handle geometry, too many primitives with the same centroid");
		CreateTempLeaf(parentIndex, childIndex, start, end, nodeBbox);
		return;
	}

	int32_t currentNode;
	int32_t leftChildIndex = childIndex;
	int32_t rightChildIndex = childIndex + NODE_WIDTH / pow(2, 1 + depth % NODE_WIDTH_LOG2);

	// Create an intermediate node if the depth indicates to do so.
	// Register the split axis.
	if (depth % NODE_WIDTH_LOG2 != NODE_WIDTH_LOG2 - 1) {
		currentNode = CreateIntermediateNode(parentIndex, childIndex, nodeBbox);
	} else {
		currentNode = parentIndex;
	}

	for (u_int i = start; i < end; i += step) {
		const u_int mIndex = meshIndexes[i];
		const u_int tIndex = triangleIndexes[i];

		// Binning is relative to the centroids bbox and to the
		// primitives' centroid.
		const int binId = Min(NB_BINS - 1, Floor2Int(k1 * (primsCentroids[mIndex][tIndex][axis] - k0)));

		bins[binId]++;
		binsBbox[binId] = Union(binsBbox[binId], primsBboxes[mIndex][tIndex]);
	}

	//--------------
	// Evaluate where to split.

	// Cumulative number of primitives in the bins from the first to the
	// ith, and from the last to the ith.
	int nbPrimsLeft[NB_BINS];
	int nbPrimsRight[NB_BINS];
	// The corresponding cumulative bounding boxes.
	BBox bboxesLeft[NB_BINS];
	BBox bboxesRight[NB_BINS];

	// The corresponding volumes.
	float vLeft[NB_BINS];
	float vRight[NB_BINS];

	BBox currentBboxLeft, currentBboxRight;
	int currentNbLeft = 0, currentNbRight = 0;

	for (int i = 0; i < NB_BINS; ++i) {
		//-----
		// Left side
		// Number of prims
		currentNbLeft += bins[i];
		nbPrimsLeft[i] = currentNbLeft;
		// Prims bbox
		currentBboxLeft = Union(currentBboxLeft, binsBbox[i]);
		bboxesLeft[i] = currentBboxLeft;
		// Surface area
		vLeft[i] = currentBboxLeft.SurfaceArea();

		//-----
		// Right side
		// Number of prims
		int rightIndex = NB_BINS - 1 - i;
		currentNbRight += bins[rightIndex];
		nbPrimsRight[rightIndex] = currentNbRight;
		// Prims bbox
		currentBboxRight = Union(currentBboxRight, binsBbox[rightIndex]);
		bboxesRight[rightIndex] = currentBboxRight;
		// Surface area
		vRight[rightIndex] = currentBboxRight.SurfaceArea();
	}

	int minBin = -1;
	float minCost = INFINITY;
	// Find the best split axis,
	// there must be at least a bin on the right side
	for (int i = 0; i < NB_BINS - 1; ++i) {
		float cost = vLeft[i] * nbPrimsLeft[i] +
				vRight[i + 1] * nbPrimsRight[i + 1];
		if (cost < minCost) {
			minBin = i;
			minCost = cost;
		}
	}

	//-----------------
	// Make the partition, in a "quicksort partitioning" way,
	// the pivot being the position of the split plane
	// (no more binId computation)
	// track also the bboxes (primitives and centroids)
	// for the left and right halves.

	// The split plane coordinate is the coordinate of the end of
	// the chosen bin along the split axis
	float splitPos = centroidsBbox.pMin[axis] + (minBin + 1) *
			(centroidsBbox.pMax[axis] - centroidsBbox.pMin[axis]) / NB_BINS;


	BBox leftChildBbox, rightChildBbox;
	BBox leftChildCentroidsBbox, rightChildCentroidsBbox;

	u_int storeIndex = start;
	for (u_int i = start; i < end; ++i) {
		const u_int mIndex = meshIndexes[i];
		const u_int tIndex = triangleIndexes[i];

		if (primsCentroids[mIndex][tIndex][axis] <= splitPos) {
			// Swap
			meshIndexes[i] = meshIndexes[storeIndex];
			meshIndexes[storeIndex] = mIndex;
			triangleIndexes[i] = triangleIndexes[storeIndex];
			triangleIndexes[storeIndex] = tIndex;
			++storeIndex;

			// Update the bounding boxes,
			// this triangle is on the left side
			leftChildBbox = Union(leftChildBbox, primsBboxes[mIndex][tIndex]);
			leftChildCentroidsBbox = Union(leftChildCentroidsBbox, primsCentroids[mIndex][tIndex]);
		} else {
			// Update the bounding boxes,
			// this triangle is on the right side.
			rightChildBbox = Union(rightChildBbox, primsBboxes[mIndex][tIndex]);
			rightChildCentroidsBbox = Union(rightChildCentroidsBbox, primsCentroids[mIndex][tIndex]);
		}
	}

	// Build recursively
	BuildTree(start, storeIndex, meshIndexes, triangleIndexes, primsBboxes, primsCentroids,
			leftChildBbox, leftChildCentroidsBbox, currentNode,
			leftChildIndex, depth + 1);
	BuildTree(storeIndex, end, meshIndexes, triangleIndexes, primsBboxes, primsCentroids,
			rightChildBbox, rightChildCentroidsBbox, currentNode,
			rightChildIndex, depth + 1);
}

/***************************************************/

void NBVHAccel::CreateTempLeaf(int32_t parentIndex, int32_t childIndex,
		u_int start, u_int end, const BBox &nodeBbox) {
	// The leaf is directly encoded in the intermediate node.
	if (parentIndex < 0) {
		// The entire tree is a leaf
		nNodes = 1;
		parentIndex = 0;
	}

	// Encode the leaf in the original way,
	// it will be transformed to a preswizzled format in a post-process.

	u_int nbPrimsTotal = end - start;

	NBVHNode &node = nodes[parentIndex];

	node.SetBBox(childIndex, nodeBbox);


	// Next multiple of 4, divided by 4
	u_int quads = (nbPrimsTotal + 3) / 4;

	// Use the same encoding as the final one, but with a different meaning.
	node.InitializeLeaf(childIndex, quads, start);

	nQuads += quads;
}

void NBVHAccel::PreSwizzle(int32_t nodeIndex, std::vector<u_int> &meshIndexes, std::vector<u_int> &triangleIndexes) {
	for (int i = 0; i < 4; ++i) {
		if (nodes[nodeIndex].ChildIsLeaf(i))
			CreateSwizzledLeaf(nodeIndex, i, meshIndexes, triangleIndexes);
		else
			PreSwizzle(nodes[nodeIndex].children[i], meshIndexes, triangleIndexes);
	}
}

void NBVHAccel::CreateSwizzledLeaf(int32_t parentIndex, int32_t childIndex,
		std::vector<u_int> &meshIndexes, std::vector<u_int> &triangleIndexes) {
	NBVHNode &node = nodes[parentIndex];
	if (node.LeafIsEmpty(childIndex))
		return;
	const u_int startQuad = nQuads;
	const u_int nbQuads = node.NbQuadsInLeaf(childIndex);

	u_int primOffset = node.FirstQuadIndexForLeaf(childIndex);
	u_int primNum = nQuads;

	for (u_int q = 0; q < nbQuads; ++q) {
		new (&prims[primNum]) QuadTriangle(meshes,
				meshIndexes[primOffset], meshIndexes[primOffset + 1], meshIndexes[primOffset + 2], meshIndexes[primOffset + 3],
				triangleIndexes[primOffset], triangleIndexes[primOffset + 1], triangleIndexes[primOffset + 2], triangleIndexes[primOffset + 3]);

		++primNum;
		primOffset += 4;
	}
	nQuads += nbQuads;
	node.InitializeLeaf(childIndex, nbQuads, startQuad);
}

/***************************************************/

bool NBVHAccel::Intersect(const Ray *initialRay, RayHit *rayHit) const {
	Ray ray(*initialRay);
	rayHit->SetMiss();

	//------------------------------
	// Prepare the ray for intersection
	QuadRay ray4(ray);
	__m128 invDir[3];
	invDir[0] = _mm_set1_ps(1.f / ray.d.x);
	invDir[1] = _mm_set1_ps(1.f / ray.d.y);
	invDir[2] = _mm_set1_ps(1.f / ray.d.z);

	int signs[3];
	ray.GetDirectionSigns(signs);

	//------------------------------
	// Main loop
	int todoNode = 0; // the index in the stack
	int32_t nodeStack[64];
	nodeStack[0] = 0; // first node to handle: root node

	while (todoNode >= 0) {
		// Leaves are identified by a negative index
		if (!NBVHNode::IsLeaf(nodeStack[todoNode])) {
			NBVHNode &node = nodes[nodeStack[todoNode]];
			--todoNode;

			// It is quite strange but checking here for empty nodes slows down the rendering
			const int32_t visit = node.BBoxIntersect(ray4, invDir, signs);

			switch (visit) {
				case (0x1 | 0x0 | 0x0 | 0x0):
					nodeStack[++todoNode] = node.children[0];
					break;
				case (0x0 | 0x2 | 0x0 | 0x0):
					nodeStack[++todoNode] = node.children[1];
					break;
				case (0x1 | 0x2 | 0x0 | 0x0):
					nodeStack[++todoNode] = node.children[0];
					nodeStack[++todoNode] = node.children[1];
					break;
				case (0x0 | 0x0 | 0x4 | 0x0):
					nodeStack[++todoNode] = node.children[2];
					break;
				case (0x1 | 0x0 | 0x4 | 0x0):
					nodeStack[++todoNode] = node.children[0];
					nodeStack[++todoNode] = node.children[2];
					break;
				case (0x0 | 0x2 | 0x4 | 0x0):
					nodeStack[++todoNode] = node.children[1];
					nodeStack[++todoNode] = node.children[2];
					break;
				case (0x1 | 0x2 | 0x4 | 0x0):
					nodeStack[++todoNode] = node.children[0];
					nodeStack[++todoNode] = node.children[1];
					nodeStack[++todoNode] = node.children[2];
					break;
				case (0x0 | 0x0 | 0x0 | 0x8):
					nodeStack[++todoNode] = node.children[3];
					break;
				case (0x1 | 0x0 | 0x0 | 0x8):
					nodeStack[++todoNode] = node.children[0];
					nodeStack[++todoNode] = node.children[3];
					break;
				case (0x0 | 0x2 | 0x0 | 0x8):
					nodeStack[++todoNode] = node.children[1];
					nodeStack[++todoNode] = node.children[3];
					break;
				case (0x1 | 0x2 | 0x0 | 0x8):
					nodeStack[++todoNode] = node.children[0];
					nodeStack[++todoNode] = node.children[1];
					nodeStack[++todoNode] = node.children[3];
					break;
				case (0x0 | 0x0 | 0x4 | 0x8):
					nodeStack[++todoNode] = node.children[2];
					nodeStack[++todoNode] = node.children[3];
					break;
				case (0x1 | 0x0 | 0x4 | 0x8):
					nodeStack[++todoNode] = node.children[0];
					nodeStack[++todoNode] = node.children[2];
					nodeStack[++todoNode] = node.children[3];
					break;
				case (0x0 | 0x2 | 0x4 | 0x8):
					nodeStack[++todoNode] = node.children[1];
					nodeStack[++todoNode] = node.children[2];
					nodeStack[++todoNode] = node.children[3];
					break;
				case (0x1 | 0x2 | 0x4 | 0x8):
					nodeStack[++todoNode] = node.children[0];
					nodeStack[++todoNode] = node.children[1];
					nodeStack[++todoNode] = node.children[2];
					nodeStack[++todoNode] = node.children[3];
					break;
			}
		} else {
			//----------------------
			// It is a leaf,
			// all the informations are encoded in the index
			const int32_t leafData = nodeStack[todoNode];
			--todoNode;

			if (NBVHNode::IsEmpty(leafData))
				continue;

			// Perform intersection
			const u_int nbQuadPrimitives = NBVHNode::NbQuadPrimitives(leafData);

			const u_int offset = NBVHNode::FirstQuadIndex(leafData);

			for (u_int primNumber = offset; primNumber < (offset + nbQuadPrimitives); ++primNumber)
				prims[primNumber].Intersect(ray4, ray, rayHit);
		}//end of the else
	}

	return !rayHit->Miss();
}

}
