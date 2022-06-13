///////////////////////////////////////////////////////////////////////////////
// RsiReaderWriterDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "RsiReaderWriterDriver.h"
#include "RsiReaderWriterClassFactory.h"

DECLARE_GENERIC_DEVICE(RSIREADERWRITERDRV)

IOSTATUS CRsiReaderWriterDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CRsiReaderWriterClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CRsiReaderWriterDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CRsiReaderWriterDriver::RSIREADERWRITERDRV_GetVersion( )
{
	return( (RSIREADERWRITERDRV_Major << 8) | RSIREADERWRITERDRV_Minor );
}

