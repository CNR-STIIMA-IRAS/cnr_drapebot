// RsiReaderWriterCtrl.cpp : Implementation of CTcRsiReaderWriterCtrl
#include "TcPch.h"
#pragma hdrstop

#include "RsiReaderWriterW32.h"
#include "RsiReaderWriterCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// CRsiReaderWriterCtrl

CRsiReaderWriterCtrl::CRsiReaderWriterCtrl() 
	: ITcOCFCtrlImpl<CRsiReaderWriterCtrl, CRsiReaderWriterClassFactory>() 
{
}

CRsiReaderWriterCtrl::~CRsiReaderWriterCtrl()
{
}

