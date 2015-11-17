#ifndef	_OPENFMC_LOG_H_
#define	_OPENFMC_LOG_H_

#include "helpers.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
	OPENFMC_LOG_ERR,
	OPENFMC_LOG_WARN,
	OPENFMC_LOG_INFO
} openfmc_log_lvl_t;

void openfmc_log(openfmc_log_lvl_t lvl, const char *fmt, ...) PRINTF_ATTR(2);
void openfmc_log_v(openfmc_log_lvl_t lvl, const char *fmt, va_list ap);

#ifdef	__cplusplus
}
#endif

#endif	/* _OPENFMC_LOG_H_ */
