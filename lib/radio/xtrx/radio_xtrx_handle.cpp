#include "radio_xtrx_handle.h"
#include <stdexcept>
#include <iostream>
#include <memory>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <string.h>


std::map<std::string, std::weak_ptr<XTRXHandle>> XTRXHandle::s_created;

std::shared_ptr<XTRXHandle> XTRXHandle::get(const std::string& name)
{
	auto idx = s_created.find(name);
	if (idx != s_created.end()) {
		 if (std::shared_ptr<XTRXHandle> obj = idx->second.lock())
			 return obj;
	}

	std::shared_ptr<XTRXHandle> obj = std::make_shared<XTRXHandle>(name);
	s_created.insert(make_pair(name, obj));
	return obj;
}

XTRXHandle::XTRXHandle(const std::string& name)
{
	int res = xtrx_open_string(name.c_str(), &_dev);
	if (res < 0)
		throw std::runtime_error(std::string("XTRXHandle::XTRXHandle(")+name.c_str()+") - unable to open the device: error: " + strerror(-res));
	devcnt = res;
}

XTRXHandle::~XTRXHandle()
{
	xtrx_close(_dev);
}



double XTRXHandle::clip_range(xtrx_direction_t dir, double rate)
{
	double ret;
	if (dir == XTRX_RX)
	{
		if (rate < 0.2e6) {
			ret = 0.2e6;
		}
		else if (rate > 56.25e6 && rate < 61.4375e6) {
			ret = 61.4375e6;
		}
		else if (rate > 80e6) {
			ret = 80e6;
		}
		else {
			ret = rate;
		}
	}
	else if (dir == XTRX_TX) {
		if (rate < 2.1e6) {
			ret = 2.1e6;
		}
		else if (rate > 56.25e6 && rate < 61.4375e6) {
			ret = 61.4375e6;
		}
		else if (rate > 80e6) {
			ret = 80e6;
		}
		else {
			ret = rate;
		}
	}

	return (double)((uint64_t)ret);
}

xtrx_channel_t XTRXHandle::xtrx_channel(int ch)
{
	if (ch == 0) {
		return XTRX_CH_A;
	}
	else if (ch == 1) {
		return XTRX_CH_B;
	}
	else {
		return XTRX_CH_AB;
	}
}