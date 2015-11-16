#ifndef	_OPENFMC_LOG_H_
#define	_OPENFMC_LOG_H_

typedef enum {
	OPENFMC_LOG_ERR,
	OPENFMC_LOG_WARN,
	OPENFMC_LOG_INFO
} openfmc_log_lvl_t;

#if	defined(__GNUC__) || defined(__clang__)
#define	PRINTF_ATTR	__attribute__ ((format (printf, 2, 3)))
#else	/* __GNUC */
#define	PRINTF_ATTR
#endif	/* __GNUC */

void openfmc_log(openfmc_log_lvl_t lvl, const char *fmt, ...) PRINTF_ATTR;
void openfmc_log_v(openfmc_log_lvl_t lvl, const char *fmt, va_list ap);

#endif	/* _OPENFMC_LOG_H_ */
