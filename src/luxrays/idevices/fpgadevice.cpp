#include <cstdio>

#include "luxrays/core/intersectiondevice.h"
#include "luxrays/core/context.h"

using namespace luxrays;

size_t FPGAIntersectionDevice::RayBufferSize = 1024;

FPGAIntersectionDevice::FPGAIntersectionDevice(const Context *context,
		const size_t devIndex) :
	HardwareIntersectionDevice(context, DEVICE_TYPE_FPGA, devIndex) {

	deviceName = std::string("FPGAIntersect");
	reportedPermissionError = false;
	rayBufferQueue = NULL;
	intersectionThread = NULL;
}

FPGAIntersectionDevice::~FPGAIntersectionDevice() {
	if (started)
		Stop();

	delete intersectionThread;
	delete rayBufferQueue;
}

void FPGAIntersectionDevice::SetDataSet(DataSet *newDataSet) {
	IntersectionDevice::SetDataSet(newDataSet);
	
	if (dataSet) {
		const AcceleratorType accelType = dataSet->GetAcceleratorType();
		if (accelType != ACCEL_AUTO) {
			accel = dataSet->GetAccelerator(accelType);
			if (!accel->CanRunOnFPGADevice(this)) {
				accel = dataSet->GetAccelerator(ACCEL_NBVH);
			}
		} else {
			accel = dataSet->GetAccelerator(ACCEL_NBVH);
		}
	}

	accel->SendSceneToFPGA();
}

void FPGAIntersectionDevice::Start() {
	IntersectionDevice::Start();

	if (dataParallelSupport) {
		rayBufferQueue = new RayBufferQueueM2O();

		// Create the thread for the rendering
		intersectionThread = new boost::thread(boost::bind(FPGAIntersectionDevice::IntersectionThread, this));

		// Set intersectionThread priority
		bool res = SetThreadRRPriority(intersectionThread);
		if (res && !reportedPermissionError) {
			LR_LOG(deviceContext, "[FPGA device::" << deviceName << "] Failed to set ray intersection thread priority (you probably need root/administrator permission to set thread realtime priority)");
			reportedPermissionError = true;
		}

		statsDeviceIdleTime = 0.0;
		statsTotalDataParallelRayCount = 0.0;
		statsDeviceTotalTime = 0.0;
	}
}

void FPGAIntersectionDevice::Interrupt() {
	assert (started);

	if (dataParallelSupport)
		intersectionThread->interrupt();
}

void FPGAIntersectionDevice::Stop() {
	IntersectionDevice::Stop();

	if (dataParallelSupport) {
		intersectionThread->interrupt();
		intersectionThread->join();
		delete intersectionThread;
		intersectionThread = NULL;

		delete rayBufferQueue;
		rayBufferQueue = NULL;
	}
}

RayBuffer *FPGAIntersectionDevice::NewRayBuffer() {
	return NewRayBuffer(RayBufferSize);
}

RayBuffer *FPGAIntersectionDevice::NewRayBuffer(const size_t size) {
	return new RayBuffer(RoundUpPow2<size_t>(size));
}

void FPGAIntersectionDevice::PushRayBuffer(RayBuffer *rayBuffer, const u_int queueIndex) {
	assert (started);
	assert (dataParallelSupport);

	rayBufferQueue->PushToDo(rayBuffer, queueIndex);
}

RayBuffer *FPGAIntersectionDevice::PopRayBuffer(const u_int queueIndex) {
	assert (started);
	assert (dataParallelSupport);

	return rayBufferQueue->PopDone(queueIndex);
}

void FPGAIntersectionDevice::IntersectionThread(FPGAIntersectionDevice *renderDevice) {
	LR_LOG(renderDevice->deviceContext, "[FPGA device::" << renderDevice->deviceName << "] Rendering thread started");

	try {
		RayBufferQueue *queue = renderDevice->rayBufferQueue;

		const double startTime = WallClockTime();
		while (!boost::this_thread::interruption_requested()) {
			const double t1 = WallClockTime();
			RayBuffer *rayBuffer = queue->PopToDo();
			renderDevice->statsDeviceIdleTime += WallClockTime() - t1;

			// Trace rays
			const Ray *rb = rayBuffer->GetRayBuffer();
			RayHit *hb = rayBuffer->GetHitBuffer();
			const size_t rayCount = rayBuffer->GetRayCount();
			
			for (unsigned int i = 0; i < rayCount; ++i) {
				hb[i].SetMiss();
				//renderDevice->accel->Intersect(&rb[i], &hb[i]);
			}

			if (xrti_intersect((void *)rb, rayCount*sizeof(Ray), rayCount, (void *)hb, rayCount*sizeof(RayHit)) < 0) {
		        FILE *f = fopen("/mnt/scratch/sam/ray_tracer/tb_generator/data/rays.txt", "w");
		        unsigned char *b = (unsigned char *)rb;
		        for (u_int i = 0; i < rayCount * sizeof(luxrays::Ray); i++) {
		            fprintf(f, "%02x", b[i]);
		            if ((i + 1) % 48 == 0)
		                fprintf(f, "\n");
		        }
		        fclose(f);
		        
		        f = fopen("/mnt/scratch/sam/ray_tracer/tb_generator/data/hits.txt", "w");
		        b = (unsigned char *)hb;
		        for (u_int i = 0; i < rayCount * sizeof(luxrays::RayHit); i++) {
		            fprintf(f, "%02x", b[i]);
		            if ((i + 1) % 20 == 0)
		                fprintf(f, "\n");
		        }
		        fclose(f);
				throw std::runtime_error("xrti_intersect failed");
			}

			renderDevice->statsTotalDataParallelRayCount += rayCount;
			queue->PushDone(rayBuffer);

			renderDevice->statsDeviceTotalTime = WallClockTime() - startTime;
		}

		LR_LOG(renderDevice->deviceContext, "[FPGA device::" << renderDevice->deviceName << "] Rendering thread halted");
	} catch (boost::thread_interrupted) {
		LR_LOG(renderDevice->deviceContext, "[FPGA device::" << renderDevice->deviceName << "] Rendering thread halted");
	}
}

//------------------------------------------------------------------------------
// Statistics
//------------------------------------------------------------------------------

double FPGAIntersectionDevice::GetLoad() const {
	return IntersectionDevice::GetLoad();
}

double FPGAIntersectionDevice::GetTotalRaysCount() const {
	return HardwareIntersectionDevice::GetTotalRaysCount();
}

double FPGAIntersectionDevice::GetTotalPerformance() const {
	return HardwareIntersectionDevice::GetTotalPerformance();
}

double FPGAIntersectionDevice::GetDataParallelPerformance() const {
	return HardwareIntersectionDevice::GetDataParallelPerformance();
}

void FPGAIntersectionDevice::ResetPerformaceStats() {
	HardwareIntersectionDevice::ResetPerformaceStats();
}