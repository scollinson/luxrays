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

#ifndef _LUXRAYS_ACCELERATOR_H
#define	_LUXRAYS_ACCELERATOR_H

#include <string>

#include "luxrays/luxrays.h"
#include "luxrays/core/trianglemesh.h"

namespace luxrays {

typedef enum {
	ACCEL_AUTO, ACCEL_BVH, ACCEL_NBVH, ACCEL_QBVH, ACCEL_MQBVH, ACCEL_MBVH
} AcceleratorType;

class OpenCLKernels;
class OpenCLIntersectionDevice;

class Accelerator {
public:
	Accelerator() { }
	virtual ~Accelerator() { }

	virtual AcceleratorType GetType() const = 0;

	virtual OpenCLKernels *NewOpenCLKernels(OpenCLIntersectionDevice *device,
		const u_int kernelCount, const u_int stackSize, const bool enableImageStorage) const = 0;
	virtual bool CanRunOnOpenCLDevice(OpenCLIntersectionDevice *device) const { return true; }

	virtual void Init(const std::deque<const Mesh *> &meshes, const u_longlong totalVertexCount, const u_longlong totalTriangleCount) = 0;
	virtual bool DoesSupportUpdate() const { return false; }
	virtual void Update() { throw new std::runtime_error("Internal error in Accelerator::Update()"); }

	virtual bool Intersect(const Ray *ray, RayHit *hit) const = 0;

	static std::string AcceleratorType2String(const AcceleratorType type);
};

}

#endif	/* _LUXRAYS_ACCELERATOR_H */
