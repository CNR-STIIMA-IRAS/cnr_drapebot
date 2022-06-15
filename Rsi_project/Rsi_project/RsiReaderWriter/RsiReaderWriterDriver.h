///////////////////////////////////////////////////////////////////////////////
// RsiReaderWriterDriver.h

#ifndef __RSIREADERWRITERDRIVER_H__
#define __RSIREADERWRITERDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define RSIREADERWRITERDRV_NAME        "RSIREADERWRITER"
#define RSIREADERWRITERDRV_Major       1
#define RSIREADERWRITERDRV_Minor       0

#define DEVICE_CLASS CRsiReaderWriterDriver

#include "ObjDriver.h"

class CRsiReaderWriterDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl RSIREADERWRITERDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(RSIREADERWRITERDRV)
	VxD_Service( RSIREADERWRITERDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __RSIREADERWRITERDRIVER_H__