#include "radio_lime_handle.h"
#include <stdexcept>
#include <iostream>
#include <memory>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <string.h>


std::map<std::string, std::weak_ptr<LimeHandle>> LimeHandle::s_created;

std::shared_ptr<LimeHandle> LimeHandle::get(lime::DeviceHandle& devHandle)
{
	auto idx = s_created.find(devHandle.ToString());
	if (idx != s_created.end()) {
		 if (std::shared_ptr<LimeHandle> obj = idx->second.lock())
			 return obj;
	}

	std::shared_ptr<LimeHandle> obj = std::make_shared<LimeHandle>(devHandle);
	s_created.insert(make_pair(devHandle.ToString(), obj));
	return obj;
}

LimeHandle::LimeHandle(lime::DeviceHandle& devHandle)
{
	_dev = lime::DeviceRegistry::makeDevice(devHandle);

	if (_dev == nullptr)
		throw std::runtime_error(std::string("LimeHandle::LimeHandle() - unable to open the device"));
	devcnt = 1;
}

LimeHandle::~LimeHandle()
{
	// !!!need to close device entirely to clear PCIE data buffers between runs, otherwise FPGA gets stuck with packets from previous run and waits for their timestamps
    // might be LitePCIE Kernel driver issue, because closing individual write/read endpoints does not clear buffers
    lime::DeviceRegistry::freeDevice(_dev);
}
