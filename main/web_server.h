#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdbool.h>
#include <stdarg.h>

void web_server_init(void);
void web_server_broadcast_log(const char *fmt, va_list args);
void web_server_update_status(int ml, int mr, bool b_fl, bool b_fr, bool ll, bool lr, bool bt_connected);

#endif
