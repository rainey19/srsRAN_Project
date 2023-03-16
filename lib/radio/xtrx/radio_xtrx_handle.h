#include <mutex>
#include <chrono>
#include <map>
#include <set>
#include <memory>

#include <xtrx_api.h>

class XTRXHandle
{
public:
	mutable std::recursive_mutex accessMutex;

	struct xtrx_dev* dev() { return _dev; }
	operator struct xtrx_dev* () { return _dev; }

	unsigned count() { return devcnt; }

	static double clip_range(xtrx_direction_t dir, double rate);

	static xtrx_channel_t xtrx_channel(int ch);
	bool get_ll(struct xtrxll_dev** lldev);

	XTRXHandle() = delete;
	XTRXHandle(const std::string& name);
	~XTRXHandle();

	xtrx_run_params_t dev_params;
	static std::shared_ptr<XTRXHandle> get(const std::string& name);

protected:
	struct xtrx_dev* _dev = NULL;
	unsigned devcnt;

	static std::map<std::string, std::weak_ptr<XTRXHandle>> s_created;
};
