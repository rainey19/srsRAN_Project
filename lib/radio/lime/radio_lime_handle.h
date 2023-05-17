#include <mutex>
#include <chrono>
#include <map>
#include <set>
#include <memory>

#include <limesuite/SDRDevice.h>
#include <limesuite/DeviceRegistry.h>
#include <limesuite/LMS7002M.h>
#include <limesuite/commonTypes.h>

class LimeHandle
{
public:
	mutable std::recursive_mutex accessMutex;

	lime::SDRDevice* dev() { return _dev; }
	operator lime::SDRDevice* () { return _dev; }

	unsigned count() { return devcnt; }

	// static double clip_range(xtrx_direction_t dir, double rate);

	// static xtrx_channel_t xtrx_channel(int ch);
	// bool get_ll(struct xtrxll_dev** lldev);

	LimeHandle() = delete;
	LimeHandle(lime::DeviceHandle& devHandle);
	~LimeHandle();

	lime::SDRDevice::SDRConfig& GetDeviceConfig() { return _dev_config; }
	lime::SDRDevice::StreamConfig& GetStreamConfig() { return _stream_config; }

	// xtrx_run_params_t dev_params;
	static std::shared_ptr<LimeHandle> get(lime::DeviceHandle& devHandle);

protected:
	lime::SDRDevice* _dev = NULL;
	lime::SDRDevice::SDRConfig _dev_config;
	lime::SDRDevice::StreamConfig _stream_config;

	unsigned devcnt;

	static std::map<std::string, std::weak_ptr<LimeHandle>> s_created;
};
