/*******************************************************************************
*                                   ZMC
*                       ----------------------------
*                          ZHIYUAN Machine Control
*
* Copyright (c) 2001-2020 Guangzhou ZHIYUAN Electronics Co., Ltd.
* All rights reserved.
*
* Contact information:
* web site:    http://www.zlg.cn/
*******************************************************************************/
 
/*----------------------------------------------------------------------------*/
/**
 * @file
 * @brief 错误码
 *
 * @internal
 * @par Modification history
 * - 1.00 2023-5-29  ZBL, 初次实现
 * @endinternal
 */
/*----------------------------------------------------------------------------*/
#ifndef ZMC_PCI_DBG_H_
#define ZMC_PCI_DBG_H_

#include <stdio.h>

#ifdef _WIN32
#define _MSG_(flt,fmt,...)          printf(flt ": %s<%d>: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define _MSG_(flt,fmt,args...)      printf(flt ": %s<%d>: " fmt "\n", __PRETTY_FUNCTION__, __LINE__, ##args)
#endif

#define _INF_(fmt,...)                  _MSG_("INF",fmt, ##__VA_ARGS__)
#define _WRN_(fmt,...)                  _MSG_("WRN",fmt, ##__VA_ARGS__)
#define _ERR_(fmt,...)                  _MSG_("ERR",fmt, ##__VA_ARGS__)
#define _DBG_(fmt,...)                  _MSG_("DBG",fmt, ##__VA_ARGS__)

#define LOG_RET(expr) { \
        int ret = (expr); if ((ret)!=0) { _ERR_(#expr " failed: err(%d):%s", ret, EcatErrorGetErrorMessage(ret)); } \
        else { _INF_(#expr " succeeded"); } \
    }

#define EXIT_IF_FAIL(expr) { \
        int ret = (expr); if ((ret)!=0) { _ERR_(#expr " failed: err(%d):%s", ret, EcatErrorGetErrorMessage(ret)); return ret; } \
    }
    
#define EXIT_IF_FAILED  EXIT_IF_FAIL

#endif  // ZMC_PCI_DBG_H_
/* end of file */
