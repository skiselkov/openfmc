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

#include "helpers.h"
#include "err.h"

/* Order must correspond to err_t */
static const char *err_name_tbl[NUM_ERRS] = {
	"OK",
	"ARPT NOT FOUND",
	"INVALID DELETE",
	"AWY/AWY MISMATCH",
	"AWY/WPT MISMATCH",
	"AWY/PROC MISMATCH",
	"WPT/PROC MISMATCH",
	"INVALID AWY",
	"DUPLICATE LEG",
	"INVALID INSERT",
	"INVALID RWY",
	"INVALID SID",
	"INVALID STAR",
	"INVALID FINAL",
	"INVALID TRANS"
};

const char *
err2str(err_t err)
{
	VERIFY(err >= 0 && err < NUM_ERRS);
	return (err_name_tbl[err]);
}
