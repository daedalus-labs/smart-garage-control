
/* SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause */
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */





#ifndef _VL53LX_PLATFORM_LOG_H_
#define _VL53LX_PLATFORM_LOG_H_


#ifdef VL53LX_LOG_ENABLE
	/** @todo Implement some form of logging  */
	#define _LOG_TRACE_PRINT(module, level, function, ...)
	#define _LOG_FUNCTION_START(module, fmt, ...)
	#define _LOG_FUNCTION_END(module, status, ...)
	#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...)
	#define _LOG_GET_TRACE_FUNCTIONS() 0
	#define _LOG_SET_TRACE_FUNCTIONS(functions)
	#define _LOG_STRING_BUFFER(x)
#else
	#define _LOG_TRACE_PRINT(module, level, function, ...)
	#define _LOG_FUNCTION_START(module, fmt, ...)
	#define _LOG_FUNCTION_END(module, status, ...)
	#define _LOG_FUNCTION_END_FMT(module, status, fmt, ...)
	#define _LOG_GET_TRACE_FUNCTIONS() 0
	#define _LOG_SET_TRACE_FUNCTIONS(functions)
	#define _LOG_STRING_BUFFER(x)
#endif

#endif

