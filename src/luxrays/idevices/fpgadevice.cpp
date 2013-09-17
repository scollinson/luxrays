#include <cstdio>

#include "luxrays/core/intersectiondevice.h"
#include "luxrays/core/context.h"

using namespace luxrays;

size_t FPGAIntersectionDevice::RayBufferSize = RAYBUFFER_SIZE;


/* Open a handle to a DRIVER device */
static WDC_DEVICE_HANDLE DeviceOpen(const WD_PCI_SLOT *pSlot)
{
    WDC_DEVICE_HANDLE hDev;
    DWORD dwStatus;
    WD_PCI_CARD_INFO deviceInfo;
    
    /* Retrieve the device's resources information */
    BZERO(deviceInfo);
    deviceInfo.pciSlot = *pSlot;
    dwStatus = WDC_PciGetDeviceInfo(&deviceInfo);
    if (WD_STATUS_SUCCESS != dwStatus)
    {
        DRIVER_ERR("DeviceOpen: Failed retrieving the device's resources information.\n"
            "Error 0x%lx\n", dwStatus);
        return NULL;
    }

    /* NOTE: You can modify the device's resources information here, if
       necessary (mainly the deviceInfo.Card.Items array or the items number -
       deviceInfo.Card.dwItems) in order to register only some of the resources
       or register only a portion of a specific address space, for example. */

    /* Open a handle to the device */
    hDev = DRIVER_DeviceOpen(&deviceInfo);
    if (!hDev)
    {
        DRIVER_ERR("DeviceOpen: Failed opening a handle to the device: %s",
            DRIVER_GetLastErr());
        return NULL;
    }

    return hDev;
}

/* Close handle to a DRIVER device */
static void DeviceClose(WDC_DEVICE_HANDLE hDev)
{
    if (!hDev)
        return;

    if (!DRIVER_DeviceClose(hDev))
    {
        DRIVER_ERR("DeviceClose: Failed closing DRIVER device: %s",
            DRIVER_GetLastErr());
    }
}

char *WDC_DIAG_IntTypeDescriptionGet(DWORD dwIntType)
{
    if (dwIntType & INTERRUPT_MESSAGE_X)
        return "Extended Message-Signaled Interrupt (MSI-X)";
    else if (dwIntType & INTERRUPT_MESSAGE)
        return "Message-Signaled Interrupt (MSI)";
    else if (dwIntType & INTERRUPT_LEVEL_SENSITIVE)
        return "Level-Sensitive Interrupt";
    return "Edge-Triggered Interrupt";
}

/* Diagnostics interrupt handler routine */
void FPGAIntersectionDevice::DiagIntHandler(WDC_DEVICE_HANDLE hDev, DRIVER_INT_RESULT *pIntResult)
{
    /* TODO: You can modify this function in order to implement your own
             diagnostics interrupt handler routine */

    printf("Got interrupt number %ld\n", pIntResult->dwCounter);
    printf("Interrupt Type: %s\n",
        WDC_DIAG_IntTypeDescriptionGet(pIntResult->dwEnabledIntType));
    if (WDC_INT_IS_MSI(pIntResult->dwEnabledIntType))
        printf("Message Data: 0x%lx\n", pIntResult->dwLastMessage);
	
}

FPGAIntersectionDevice::FPGAIntersectionDevice(const Context *context,
		WD_PCI_SLOT slot, const size_t devIndex) :
	HardwareIntersectionDevice(context, DEVICE_TYPE_FPGA, devIndex) {
	
	hDev = DeviceOpen(&slot);
    
	DRIVER_DMAOpen(hDev);
	// Assert Initiator Reset 
    DRIVER_WriteDCSR(hDev, 0x1);
    // De-assert Initiator Reset 
    DRIVER_WriteDCSR(hDev, 0x0);
	// Disable Interrupts
	DRIVER_WriteDDMACR(hDev, BIT7 | BIT23);

	char buf[64];
	sprintf(buf, "FPGAIntersectThread-%03d", (int)devIndex);
	deviceName = std::string(buf);
	reportedPermissionError = false;
	externalRayBufferQueue = NULL;
	intersectionThread = NULL;
}

FPGAIntersectionDevice::~FPGAIntersectionDevice() {
	PDRIVER_DEV_CTX pDevCtx;

	if (started)
		Stop();
	delete intersectionThread;

	if (hDev) {
		pDevCtx = (PDRIVER_DEV_CTX)WDC_GetDevContext(hDev);
		DRIVER_DMAClose(pDevCtx->hDma);
		DeviceClose(hDev);
	}
}

void FPGAIntersectionDevice::SetExternalRayBufferQueue(RayBufferQueue *queue) {
	assert (!started);

	externalRayBufferQueue = queue;
}

void FPGAIntersectionDevice::SetDataSet(DataSet *newDataSet) {
	IntersectionDevice::SetDataSet(newDataSet);

	dataSet->GetAccelerator(ACCEL_NBVH)->SendToFPGA(hDev);
}

void FPGAIntersectionDevice::Start() {
	IntersectionDevice::Start();

	if (dataParallelSupport) {
		// Create the thread for the rendering
		intersectionThread = new boost::thread(boost::bind(FPGAIntersectionDevice::IntersectionThread, this));

		// Set intersectionThread priority
		bool res = SetThreadRRPriority(intersectionThread);
		if (res && !reportedPermissionError) {
			LR_LOG(deviceContext, "[FPGA device::" << deviceName << "] Failed to set ray intersection thread priority (you probably need root/administrator permission to set thread realtime priority)");
			reportedPermissionError = true;
		}
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

		if (!externalRayBufferQueue)
			rayBufferQueue.Clear();
	}
}

RayBuffer *FPGAIntersectionDevice::NewRayBuffer() {
	return NewRayBuffer(RayBufferSize);
}

RayBuffer *FPGAIntersectionDevice::NewRayBuffer(const size_t size) {
	return new RayBuffer(RoundUpPow2<size_t>(size));
}

void FPGAIntersectionDevice::PushRayBuffer(RayBuffer *rayBuffer) {
	assert (started);
	assert (!externalRayBufferQueue);
	assert (dataParallelSupport);

	rayBufferQueue.PushToDo(rayBuffer, 0);
}

RayBuffer *FPGAIntersectionDevice::PopRayBuffer() {
	assert (started);
	assert (!externalRayBufferQueue);
	assert (dataParallelSupport);

	return rayBufferQueue.PopDone(0);
}

void FPGAIntersectionDevice::IntersectionThread(FPGAIntersectionDevice *renderDevice) {
	LR_LOG(renderDevice->deviceContext, "[FPGA device::" << renderDevice->deviceName << "] Rendering thread started");
    DWORD dwStatus;
    PDRIVER_DEV_CTX pDevCtx = (PDRIVER_DEV_CTX)WDC_GetDevContext(renderDevice->hDev);
    DWORD dwOptions = DMA_TO_DEVICE | DMA_KERNEL_BUFFER_ALLOC;
    UINT32 RayAddr, HitAddr;

    /* Allocate and lock a DMA buffer */
    dwStatus = WDC_DMAContigBufLock(renderDevice->hDev, &pDevCtx->pRayBuf, dwOptions, RayBufferSize * sizeof(Ray),
        &pDevCtx->hDma->pRayDma);
    if (WD_STATUS_SUCCESS != dwStatus) 
    {
        DRIVER_ERR("DRIVER_DMAOpen: Failed locking a DMA buffer. "
            "Error 0x%lx\n", dwStatus);
        goto Error;
    }
    
    dwOptions = DMA_FROM_DEVICE | DMA_KERNEL_BUFFER_ALLOC;
    dwStatus = WDC_DMAContigBufLock(renderDevice->hDev, &pDevCtx->pHitBuf, dwOptions, RayBufferSize * sizeof(RayHit),
        &pDevCtx->hDma->pHitDma);
    if (WD_STATUS_SUCCESS != dwStatus) 
    {
        DRIVER_ERR("DRIVER_DMAOpen: Failed locking a DMA buffer. "
            "Error 0x%lx\n", dwStatus);
        goto Error;
    }
    
    pDevCtx->fIsRead = FALSE;

    RayAddr = (UINT32)pDevCtx->hDma->pRayDma->Page[0].pPhysicalAddr;
    DRIVER_WriteRAYADDR(renderDevice->hDev, RayAddr);

    HitAddr = (UINT32)pDevCtx->hDma->pHitDma->Page[0].pPhysicalAddr;
    DRIVER_WriteHITADDR(renderDevice->hDev, HitAddr);

	try {
		RayBufferQueue *queue = renderDevice->externalRayBufferQueue ?
			renderDevice->externalRayBufferQueue : &(renderDevice->rayBufferQueue);

		const double startTime = WallClockTime();
		while (!boost::this_thread::interruption_requested()) {
			const double t1 = WallClockTime();
			RayBuffer *rayBuffer = queue->PopToDo();
			renderDevice->statsDeviceIdleTime += WallClockTime() - t1;
			// Trace rays
			const Ray *rb = rayBuffer->GetRayBuffer();
			RayHit *hb = rayBuffer->GetHitBuffer();
			const size_t rayCount = rayBuffer->GetRayCount();
			
			DRIVER_WriteRAYCOUNT(renderDevice->hDev, rayCount);
			
			memcpy(pDevCtx->pRayBuf, rb, rayCount * sizeof(Ray));
			
			DRIVER_DMAStart(pDevCtx->hDma, pDevCtx->fIsRead);
			DRIVER_DMAPollCompletion(pDevCtx->hDma, pDevCtx->fIsRead);
			
			memcpy(hb, pDevCtx->pHitBuf, rayCount * sizeof(RayHit));
			
			renderDevice->statsTotalDataParallelRayCount += rayCount;
			queue->PushDone(rayBuffer);

			renderDevice->statsDeviceTotalTime = WallClockTime() - startTime;
		}

		LR_LOG(renderDevice->deviceContext, "[FPGA device::" << renderDevice->deviceName << "] Rendering thread halted");
	} catch (boost::thread_interrupted) {
		LR_LOG(renderDevice->deviceContext, "[FPGA device::" << renderDevice->deviceName << "] Rendering thread halted");
	}
Error:
    if (pDevCtx->hDma)
        DRIVER_DMAClose((DRIVER_DMA_HANDLE)pDevCtx->hDma);
}
