/****************************************************************************
 * fs/inode/fs_inoderelease.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_release
 *
 * Description:
 *   This is called from close() logic when it no longer refers to the inode.
 *
 ****************************************************************************/

void inode_release(FAR struct inode *inode)
{
  if (inode)
    {
      /* Decrement the references of the inode */

      inode_lock();

      if (inode->i_crefs)
        {
          inode->i_crefs--;
        }

      /* If the subtree was previously deleted and the reference
       * count has decrement to zero,  then delete the inode
       * now.
       */

      if (inode->i_crefs <= 0)
        {
          /* If the inode has been properly unlinked, then the peer pointer
           * should be NULL.
           */

          inode_unlock();

          DEBUGASSERT(inode->i_peer == NULL);
          inode_free(inode);
        }
      else
        {
          inode_unlock();
        }
    }
}
