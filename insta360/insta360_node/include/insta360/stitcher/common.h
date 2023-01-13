#ifndef INSTA360_COMMOM_H__
#define INSTA360_COMMOM_H__
#include <functional>
#define INSTA360_NAMESPACE ins_media

enum class STITCH_ERR {
	NoMetaData,
	NoSupportedFormat,
	InitPipelineFailed,
	FlowStateInitFailed,
	BlenderInitFailed,
	ColorPlusInitFailed,
	DenoiseInitFailed,
	StitchFusionInitFailed,
	BlendError,
	InvaildParam = -100,
	NoError = 0,
};

enum class HDR_TYPE {
	ImageHdr_NONE = -1,
	SingleImageHdr = 0,
	MultiImageHdr_mbb = 1,
	MultiImageHdr_mpl = 2,
};

enum class STITCH_TYPE {
	TEMPLATE,
	OPTFLOW,
	DYNAMICSTITCH,
};

typedef std::function<void(int process, int error)> stitch_process_callback;
typedef std::function<void(int error, const char* errinfo)> stitch_error_callback;

#endif