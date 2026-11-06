#include <rtthread.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
#endif

#include "status.h"
#include "flight_thread_app.h"

static const char *flight_status_name(flight_status_t status)
{
    switch (status)
    {
    case FLIGHT_STATUS_STOPPED:
        return "stopped";
    case FLIGHT_STATUS_INITIALIZING:
        return "initializing";
    case FLIGHT_STATUS_RUNNING:
        return "running";
    case FLIGHT_STATUS_FAILSAFE:
        return "failsafe";
    default:
        return "unknown";
    }
}

static int fypilot_status(int argc, char **argv)
{
    RT_UNUSED(argc);
    RT_UNUSED(argv);

    rt_kprintf("FyPilot: status=%s, loops=%u\n",
               flight_status_name(get_status()),
               (unsigned int)get_loop_count());
    return 0;
}
MSH_CMD_EXPORT(fypilot_status, show FyPilot runtime status);
