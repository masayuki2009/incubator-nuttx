/****************************************************************************
 * libs/libc/stdlib/lib_itoa.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2010-2011 Gregory Nutt
 * SPDX-FileCopyrightText: 2013 Brooks Automation, Inc. All rights reserved.
 * SPDX-FileContributor: Ryan Sundberg <ryan.sundberg@brooks.com>
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdlib.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR char *itoa(int val, FAR char *str, int base)
{
  static FAR const char *digits = "0123456789abcdefghijklmnopqrstuvwxyz";
  int intval = abs(val);
  int digit;
  int pos;
  int len;
  FAR char *buf = str;
  char swap;

  if (base >= 2 && base <= 36)
    {
      do
        {
          digit = intval % base;
          intval = intval / base;
          *buf++ = digits[digit];
        }
      while (intval > 0);

      if (val < 0)
        {
          *buf++ = '-';
        }

      for (pos = 0, len = buf - str; pos < len / 2; pos++)
        {
          swap = str[len - pos - 1];
          str[len - pos - 1] = str[pos];
          str[pos] = swap;
        }
    }

  *buf = '\0';

  return str;
}
