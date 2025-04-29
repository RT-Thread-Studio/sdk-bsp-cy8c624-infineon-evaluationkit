/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-4-27      Hydevcode    first version
 */
#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "app.filesystem"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#ifdef BSP_USING_TFCARD
static int filesystem_mount(void)
{
    /* 挂载 TF 卡中的文件系统，参数 elm 表示挂载的文件系统类型为 elm-fat 文件系统*/
    if (dfs_mount("sd0", "/", "elm", 0, 0) == 0)
    {
        LOG_I("Filesystem initialized!");
    }
    else
    {
        LOG_E("Failed to initialize filesystem!");
    }
    return 0;
}
INIT_APP_EXPORT(filesystem_mount);
#endif /*BSP_USING_TFCARD*/
