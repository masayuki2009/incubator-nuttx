/****************************************************************************
 * libs/libm/libm/lib_tanhl.c
 *
 * SPDX-License-Identifier: ISC
 * SPDX-FileCopyrightText: Copyright (C) 2012 Gregory Nutt.
 * SPDX-FileContributor: Ported by: Darcy Gong
 *
 * It derives from the Rhombus OS math library by Nick Johnson which has
 * a compatible, MIT-style license:
 *
 * Copyright (C) 2009, 2010 Nick Johnson <nickbjohnson4224 at gmail.com>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <math.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_HAVE_LONG_DOUBLE
long double tanhl(long double x)
{
  long double x0 = exp(x);
  long double x1 = 1.0 / x0;

  return ((x0 - x1) / (x0 + x1));
}
#endif
