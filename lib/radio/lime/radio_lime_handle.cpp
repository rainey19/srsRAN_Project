/*
 *
 * Copyright 2021-2024 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

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
	// NOT USING THIS FOR NOW
	// free(GetStreamConfig().userData);
	
	// !!!need to close device entirely to clear PCIE data buffers between runs, otherwise FPGA gets stuck with packets from previous run and waits for their timestamps
    // might be LitePCIE Kernel driver issue, because closing individual write/read endpoints does not clear buffers
    lime::DeviceRegistry::freeDevice(_dev);
}

bool make_arg_pair(std::string arg, std::pair<std::string, std::string>& pair)
{
	try
	{
		size_t x    = arg.find("=");
		pair.first  = arg.substr(0, x);
		pair.second = arg.substr(x+1);
		return true;
	}
	catch (...)
	{
		printf("Error parsing argument: %s\n", arg.c_str());
		return false;
	}
}

bool LimeHandle::split_args(std::string args, std::vector<std::pair<std::string, std::string>>& arg_list)
{
	std::string _store_str;
	std::stringstream _stringstream(args);

	while (getline(_stringstream, _store_str, ','))
	{
		std::pair<std::string, std::string> pair;
		if (!make_arg_pair(_store_str, pair))
		{
		printf("Could not parse radio args!\n");
		return false;
		}
		arg_list.push_back(pair);
	}
	return true;
}