#include <hexo/power.h>

error_t power_reboot()
{
	return ENOTSUP;
}

error_t power_shutdown()
{
	return ENOTSUP;
}

enum power_reset_cause_e power_reset_cause(void)
{
	return POWER_RESET_CAUSE_UNKNOWN;
}
