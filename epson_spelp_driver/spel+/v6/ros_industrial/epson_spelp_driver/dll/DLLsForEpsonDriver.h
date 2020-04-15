// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the DLLSFOREPSONDRIVER_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// DLLSFOREPSONDRIVER_API functions as being imported from a DLL, whereas this DLL sees symbols
// defined with this macro as being exported.
#ifdef DLLSFOREPSONDRIVER_EXPORTS
#define DLLSFOREPSONDRIVER_API __declspec(dllexport)
#else
#define DLLSFOREPSONDRIVER_API __declspec(dllimport)
#endif

// This class is exported from the DLLsForEpsonDriver.dll
class DLLSFOREPSONDRIVER_API CDLLsForEpsonDriver {
public:
	CDLLsForEpsonDriver(void);
	// TODO: add your methods here.
};

extern DLLSFOREPSONDRIVER_API int nDLLsForEpsonDriver;

DLLSFOREPSONDRIVER_API int fnDLLsForEpsonDriver(void);