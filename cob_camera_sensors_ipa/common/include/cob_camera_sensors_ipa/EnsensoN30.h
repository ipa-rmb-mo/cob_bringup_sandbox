// Drivers for EnsensoN30 camera:
// http://www.ensenso.com/support/sdk-download/
// follow instructions in the "README" File under Installation Notes 
//
// ROS example
// https://github.com/PointCloudLibrary/pcl/blob/master/doc/tutorials/content/ensenso_cameras.rst
//
//
//---------------------------------------------------------------------
// the "main" calling the EnsensoN30 could look like this:
//
//int main(int argc, char* argv[])
//{
//	ipa_CameraSensors::EnsensoN30 ensenso_n30;
//	ensenso_n30.Init("C:\ensenso_tests\ ",1);
//	ensenso_n30.Open();
//	cv::Mat ensensoRangeimage;	
//	cv::Mat ensensoRangeimage_8U3;
//	cv::Mat ensensoColorimage;	
//
//	int cmd = 0;
//	while (cmd != 'q')
//	{
//		ensenso_n30.AcquireImages(&ensensoRangeimage,&ensensoColorimage);//, cv::Mat* grayImage, cv::Mat* cartesianImage,		bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
//	
//		ipa_Utils::ConvertToShowImage(ensensoRangeimage, ensensoRangeimage_8U3);
//		cv::imshow("Range", ensensoRangeimage_8U3);
//		cv::imshow("Color", ensensoColorimage);
//		cmd = cv::waitKey(20);
//	}
//}


#ifndef __IPA_ENSENSO_N30_H__
#define __IPA_ENSENSO_N30_H__

#ifdef __LINUX__
	#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#else
	#include <cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractRangeImagingSensor.h>
#endif

#include "nxLib.h"

namespace ipa_CameraSensors {

/// @ingroup RangeCameraDriver
/// Platform independent interface to EnsensoN30 camera.
class __DLL_LIBCAMERASENSORS__ EnsensoN30 : public AbstractRangeImagingSensor
{
public:

	enum t_EnsensoN30VideoFormat
	{
		SXGA = 0, // 1280×1024
		VGA //640x480
	};

	EnsensoN30();
	~EnsensoN30();

	//*******************************************************************************
	// AbstractRangeImagingSensor interface implementation
	//*******************************************************************************

	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	unsigned long Close();

	unsigned long SetProperty(t_cameraProperty* cameraProperty);
	unsigned long SetPropertyDefaults();
	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* RangeImage=NULL, char* IntensityImage=NULL,
		char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);
	unsigned long AcquireImages(cv::Mat* rangeImage = 0, cv::Mat* intensityImage = 0,
		cv::Mat* cartesianImage = 0, bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);

	/// Returns the camera type.
	/// @return The camera type
	t_cameraType GetCameraType() { return m_CameraType; }

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}

private:
	
	NxLibItem m_Camera;					///< accessor element to the opened Ensenso camera

	t_cameraType m_CameraType;			///< Camera Type

	t_EnsensoN30VideoFormat m_VideoFormat; ///< Video format of color camera

	//*******************************************************************************
	// Camera specific members
	//*******************************************************************************
	
	std::string m_Serial;		///< serial number of the device

	unsigned long LoadParameters(const char* filename, int cameraIndex);
};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr CreateRangeImagingSensor_EnsensoN30();

} // End namespace ipa_CameraSensors
#endif // __IPA_ENSENSO_N30_H__