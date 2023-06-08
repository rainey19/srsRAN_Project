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

	LimeHandle() = delete;
	LimeHandle(lime::DeviceHandle& devHandle);
	~LimeHandle();

	lime::SDRDevice::SDRConfig& GetDeviceConfig() { return _dev_config; }
	lime::SDRDevice::StreamConfig& GetStreamConfig() { return _stream_config; }

	static std::shared_ptr<LimeHandle> get(lime::DeviceHandle& devHandle);

	static bool split_args(std::string args, std::vector<std::pair<std::string, std::string>>& arg_list);

protected:
	lime::SDRDevice* _dev = NULL;
	lime::SDRDevice::SDRConfig _dev_config;
	lime::SDRDevice::StreamConfig _stream_config;

	unsigned devcnt;

	static std::map<std::string, std::weak_ptr<LimeHandle>> s_created;
};
