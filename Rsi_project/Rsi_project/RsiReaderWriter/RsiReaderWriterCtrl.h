///////////////////////////////////////////////////////////////////////////////
// RsiReaderWriterCtrl.h

#ifndef __RSIREADERWRITERCTRL_H__
#define __RSIREADERWRITERCTRL_H__

#include <atlbase.h>
#include <atlcom.h>


#include "resource.h"       // main symbols
#include "RsiReaderWriterW32.h"
#include "TcBase.h"
#include "RsiReaderWriterClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CRsiReaderWriterCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CRsiReaderWriterCtrl, &CLSID_RsiReaderWriterCtrl>
	, public IRsiReaderWriterCtrl
	, public ITcOCFCtrlImpl<CRsiReaderWriterCtrl, CRsiReaderWriterClassFactory>
{
public:
	CRsiReaderWriterCtrl();
	virtual ~CRsiReaderWriterCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_RSIREADERWRITERCTRL)
DECLARE_NOT_AGGREGATABLE(CRsiReaderWriterCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CRsiReaderWriterCtrl)
	COM_INTERFACE_ENTRY(IRsiReaderWriterCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __RSIREADERWRITERCTRL_H__
