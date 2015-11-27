/*
 * CDDL HEADER START
 *
 * The contents of this file are subject to the terms of the
 * Common Development and Distribution License, Version 1.0 only
 * (the "License").  You may not use this file except in compliance
 * with the License.
 *
 * You can obtain a copy of the license at usr/src/OPENSOLARIS.LICENSE
 * or http://www.opensource.org/licenses/CDDL-1.0.
 * See the License for the specific language governing permissions
 * and limitations under the License.
 *
 * When distributing Covered Code, include this CDDL HEADER in each
 * file and include the License file at usr/src/OPENSOLARIS.LICENSE.
 * If applicable, add the following below this CDDL HEADER, with the
 * fields enclosed by brackets "[]" replaced with your own identifying
 * information: Portions Copyright [yyyy] [name of copyright owner]
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2015 Saso Kiselkov. All rights reserved.
 */

#ifndef	_OPENFMC_ERR_H_
#define	_OPENFMC_ERR_H_

typedef enum {
	ERR_OK = 0,
	ERR_ARPT_NOT_FOUND,
	ERR_INVALID_DELETE,
	ERR_AWY_AWY_MISMATCH,
	ERR_AWY_WPT_MISMATCH,
	ERR_AWY_PROC_MISMATCH,
	ERR_WPT_PROC_MISMATCH,
	ERR_AWY_NOT_FOUND,
	ERR_DUPLICATE_LEG,
	ERR_INVALID_INSERT,
	NUM_ERRS
} err_t;

const char *err2str(err_t err);

#endif	/* _OPENFMC_ERR_H_ */
