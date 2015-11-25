#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/EnsensoN30.h"	
	#include "cob_vision_utils/GlobalDefines.h"

	#include "tinyxml.h"
	#include <fstream>
	#include <iostream>
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/EnsensoN30.h"
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif



using namespace ipa_CameraSensors;
#define AVG(a,b) (((int)(a) + (int)(b)) >> 1)
#define AVG3(a,b,c) (((int)(a) + (int)(b) + (int)(c)) / 3)
#define AVG4(a,b,c,d) (((int)(a) + (int)(b) + (int)(c) + (int)(d)) >> 2)
#define WAVG4(a,b,c,d,x,y)  ( ( ((int)(a) + (int)(b)) * (int)(x) + ((int)(c) + (int)(d)) * (int)(y) ) / ( 2 * ((int)(x) + (int(y))) ) )
#define IPA_CLIP_CHAR(c) ((c)>255?255:(c)<0?0:(c))
#define IDS_SXGA_X_RES 1280
#define IDS_SXGA_Y_RES 1024
#define IDS_VGA_X_RES 640
#define IDS_VGA_Y_RES 480

__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr ipa_CameraSensors::CreateRangeImagingSensor_EnsensoN30()
{
	return AbstractRangeImagingSensorPtr(new EnsensoN30());
}

EnsensoN30::EnsensoN30()
	: m_Camera("")
{
	m_initialized = false;
	m_open = false;

	m_BufferSize = 1;
}

EnsensoN30::~EnsensoN30()
{
	if (isOpen())
		Close();

	if (isInitialized())
	{
		try
		{
			nxLibFinalize();
		}
		catch (NxLibException ex)
		{
			std::cerr << "ERROR - EnsensoN30::Init:" << std::endl;
			std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
		}
	}
}


unsigned long EnsensoN30::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return ipa_Utils::RET_OK;
	}

	// Load camera parameters from xml-file
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - EnsensoN30::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	try
	{
		std::cout << "Opening NxLib and waiting for cameras to be detected\n";
		nxLibInitialize(true);
	}
	catch (NxLibException ex)
	{
		std::cerr << "ERROR - EnsensoN30::Init:" << std::endl;
		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
	}

	//switch (m_ColorCamVideoFormat)
	//{
	//case SXGA:
	//	m_output_mode.setFps(30);
	//	m_output_mode.setResolution(IDS_SXGA_X_RES,IDS_SXGA_Y_RES); //Resolution of SXGA : 1280*1024
	//	std::cout << "INFO - ColorCamVideoFormat: SXGA" << std::endl;
	//	break;
	//case VGA:
	//	m_output_mode.setFps(30);
	//	m_output_mode.setResolution(IDS_VGA_X_RES,IDS_VGA_Y_RES); //Resolution of VGA : 640*480
	//	std::cout << "INFO - ColorCamVideoFormat: VGA" << std::endl;
	//	break;
	//default:
	//	m_output_mode.setFps(30);
	//	m_output_mode.setResolution(IDS_SXGA_X_RES,IDS_SXGA_Y_RES); //Resolution of SXGA : 1280*1024
	//	std::cout << "INFO - ColorCamVideoFormat: Default" << std::endl;
	//}

	m_CameraType = ipa_CameraSensors::CAM_ENSENSO_N30;

	// Set init flag
	m_initialized = true;

	return ipa_Utils::RET_OK;
}


unsigned long EnsensoN30::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}
	
	try
	{
		NxLibItem root; // Reference to the API tree root

		// Create an object referencing the camera's tree item, for easier access:
		m_Camera = root[itmCameras][itmBySerialNo][m_Serial];
		if (!m_Camera.exists() || (m_Camera[itmType] != valStereo))
		{
			std::cerr << "ERROR - EnsensoN30::Init:" << std::endl;
			std::cerr << "\t ... Please connect an Ensenso N30 stereo camera to your computer." << std::endl;
		}

		std::string serial = m_Camera[itmSerialNumber].asString();
		std::cout << "Opening camera " << serial << std::endl;
		NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
		open.parameters()[itmCameras] = serial; // Set parameters for the open command
		open.execute();
	}
	catch (NxLibException ex)
	{
		std::cerr << "ERROR - EnsensoN30::Open:" << std::endl;
		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
	}

	std::cout << "*************************************************" << std::endl;
	std::cout << "EnsensoN30::Open: EnsensoN30 camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long EnsensoN30::Close()
{
	if (isOpen() == false)
		return RET_OK;

	std::cout << "INFO - EnsensoN30: Closing device..." << std::endl;

	try
	{
		NxLibCommand close(cmdClose);
		close.parameters()[itmSerialNumber] = m_Serial;
		close.execute();
		m_Camera = NxLibItem("");
	}
	catch (NxLibException ex)
	{
		std::cerr << "ERROR - EnsensoN30::Close:" << std::endl;
		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
	}

	m_open = false;
	return RET_OK;
}


unsigned long EnsensoN30::SetProperty(t_cameraProperty* cameraProperty) 
{
	return ipa_Utils::RET_OK;
}


unsigned long EnsensoN30::SetPropertyDefaults() 
{
	return ipa_Utils::RET_OK;
}


unsigned long EnsensoN30::GetProperty(t_cameraProperty* cameraProperty) 
{
	int ret = 0;
	
	switch (cameraProperty->propertyID)
	{
	case PROP_CAMERA_RESOLUTION:
		cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
		if (isOpen())
		{
			// todo: adapt if necessary
			// Depth image is upsampled according to the size of the color image
			cameraProperty->cameraResolution.xResolution = m_Camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0].asInt() - m_Camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0].asInt();
			cameraProperty->cameraResolution.yResolution = m_Camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1].asInt() - m_Camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1].asInt();
		}
		else
		{
			std::cout << "WARNING - EnsensoN30::GetProperty:" << std::endl;
			std::cout << "\t ... Camera not open" << std::endl;
			std::cout << "\t ... Returning default width and height of '" << IDS_SXGA_X_RES << "' x '" << IDS_SXGA_Y_RES << "'" << std::endl;
			cameraProperty->cameraResolution.xResolution = IDS_SXGA_X_RES;
			cameraProperty->cameraResolution.yResolution = IDS_SXGA_Y_RES;
		}
		break;

	default: 				
		std::cout << "ERROR - EnsensoN30::GetProperty:" << std::endl;
		std::cout << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
		return ipa_Utils::RET_FAILED;
		break;
	}

	return ipa_Utils::RET_OK;
}


unsigned long EnsensoN30::AcquireImages(cv::Mat* rangeImage, cv::Mat* grayImage, cv::Mat* cartesianImage,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	char* rangeImageData = 0;
	char* grayImageData = 0;
	char* cartesianImageData = 0;
	int widthStepRange = -1;
	int widthStepGray = -1;
	int widthStepCartesian = -1;

	//int color_width = m_image_md.XRes();
	//int color_height = m_image_md.YRes();

	int color_width = IDS_SXGA_X_RES;	//m_vs_rgb.getVideoMode().getResolutionX();
	int color_height = IDS_SXGA_Y_RES;	//m_vs_rgb.getVideoMode().getResolutionY();

	if(rangeImage)
	{
		// Depth image is upsampled according to the size of the color image
		rangeImage->create(color_height, color_width, CV_32FC1);
		rangeImageData = rangeImage->ptr<char>(0);
		widthStepRange = rangeImage->step;
	}
	
	if(grayImage)
	{
		grayImage->create(color_height, color_width, CV_8UC1);
		grayImageData = grayImage->ptr<char>(0);
		widthStepGray = grayImage->step;
	}	

	if(cartesianImage)
	{
		// Depth image is upsampled according to the size of the color image
		cartesianImage->create(color_height, color_width, CV_32FC3);
		cartesianImageData = cartesianImage->ptr<char>(0);
		widthStepCartesian = cartesianImage->step;
	}

	if (!rangeImage && !grayImage && !cartesianImage)
		return RET_OK;

	return AcquireImages(widthStepRange, widthStepGray, widthStepCartesian, rangeImageData, grayImageData,  cartesianImageData, getLatestFrame, undistort, grayImageType);
}

unsigned long EnsensoN30::AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* rangeImageData, char* grayImageData, char* cartesianImageData,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	// point map z --> range image
	// point map --> cartesian image
	// rectified left --> gray image

	// retrieve point cloud and left camera image (left camera = master)
	try
	{
		// execute the 'Capture', 'ComputeDisparityMap' and 'ComputePointMap' commands
		// grab an image
		NxLibCommand capture(cmdCapture);
		capture.parameters()[itmCameras] = m_Serial;
		capture.execute();

		// compute the disparity map, this is the actual, computation intensive stereo matching task
		NxLibCommand computeDisparity(cmdComputeDisparityMap);
		computeDisparity.parameters()[itmCameras] = m_Serial;
		computeDisparity.execute();

		// generating point map from disparity map, this converts the disparity map into XYZ data for each pixel
		NxLibCommand computePointMap(cmdComputePointMap);
		computePointMap.parameters()[itmCameras] = m_Serial;
		computePointMap.execute();

		// get info about the computed point map and copy it into a std::vector
		std::vector<float> pointMap;
		int range_width=0, range_height=0;
		m_Camera[itmImages][itmPointMap].getBinaryDataInfo(&range_width, &range_height, 0,0,0,0);
		m_Camera[itmImages][itmPointMap].getBinaryData(pointMap, 0);
		if (cartesianImageData)
		{
			const int number_elements = range_height*range_width;
			float* p_cartesianImageData = (float*)cartesianImageData;
			for (int i=0; i<number_elements; ++i)
			{
				*p_cartesianImageData = 0.001f * pointMap[3*i];
				++p_cartesianImageData;
				*p_cartesianImageData = 0.001f * pointMap[3*i+1];
				++p_cartesianImageData;
				*p_cartesianImageData = 0.001f * pointMap[3*i+2];
				++p_cartesianImageData;
			}
		}
		if (rangeImageData)
		{
			const int number_elements = range_height*range_width;
			float* p_rangeImageData = (float*)rangeImageData;
			for (int i=0; i<number_elements; ++i)
			{
				*p_rangeImageData = 0.001f * pointMap[3*i+2];
				++p_rangeImageData;
			}
		}

		// get the intensity image
		std::vector<unsigned char> imageLeft;
		int color_width=0, color_height=0;
		m_Camera[itmImages][itmRectified][itmLeft].getBinaryDataInfo(&color_width, &color_height, 0,0,0,0);
		m_Camera[itmImages][itmRectified][itmLeft].getBinaryData(imageLeft, 0);
		if (grayImageData)
		{
			memcpy((unsigned char*)grayImageData, &(imageLeft[0]), color_width * color_height * sizeof(unsigned char) * 1);
		}
	}
	catch (NxLibException ex)
	{
		std::cerr << "ERROR - EnsensoN30::AcquireImages:" << std::endl;
		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
	}

	return  RET_OK;
}


unsigned long EnsensoN30::SaveParameters(const char* filename) 
{
	return ipa_Utils::RET_OK;
}


unsigned long EnsensoN30::LoadParameters(const char* filename, int cameraIndex)
{
	// Load EnsensoN30 parameters.
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - EnsensoN30::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... '" << filename << "'" << std::endl;

	std::string tempString;
	if ( p_configXmlDocument )
	{
//************************************************************************************
//	BEGIN LibCameraSensors
//************************************************************************************
		// Tag element "LibCameraSensors" of Xml Inifile		
		TiXmlElement *p_xmlElement_Root = NULL;
		p_xmlElement_Root = p_configXmlDocument->FirstChildElement( "LibCameraSensors" );

		if ( p_xmlElement_Root )
		{
//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoN30
//************************************************************************************
			// Tag element "EnsensoN30 of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_Ensenso = NULL;
			std::stringstream ss;
			ss << "EnsensoN30_" << cameraIndex;
			p_xmlElement_Root_Ensenso = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_Ensenso )
			{
//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoN30->Serial
//************************************************************************************
				// Subtag element "Serial" of XML Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Ensenso->FirstChildElement( "Serial" );
				if ( p_xmlElement_Child )
				{
					m_Serial = "";
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Serial'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					m_Serial = tempString;
				}
				else
				{
					std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Serial'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoN30->Role
//************************************************************************************
				// Subtag element "Role" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Ensenso->FirstChildElement( "Role" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Role'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					if (tempString == "MASTER") m_RangeCameraParameters.m_CameraRole = MASTER;
					else if (tempString == "SLAVE") m_RangeCameraParameters.m_CameraRole = SLAVE;
					else
					{
						std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
						std::cerr << "\t ... Role " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Role'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoN30->VideoFormat
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Ensenso->FirstChildElement( "VideoFormat" );
				std::string tempString;
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "type", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'type' of tag 'VideoFormat'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "SXGA") 
					{
						m_VideoFormat = SXGA;
					}
					//else if (tempString == "VGA")
					//{
					//	m_VideoFormat = VGA;
					//}
					else
					{
						std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
						std::cerr << "\t ... Video format " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'VideoFormat'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoN30->CalibrationMethod
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Ensenso->FirstChildElement( "CalibrationMethod" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "name", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'name' of tag 'CalibrationMethod'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (tempString == "MATLAB") 
					{
						m_CalibrationMethod = MATLAB;
					}
					else if (tempString == "MATLAB_NO_Z")
					{
						m_CalibrationMethod = MATLAB_NO_Z;
					}
					else if (tempString == "NATIVE") 
					{
						m_CalibrationMethod = NATIVE;
					}
					else
					{
						std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
						std::cerr << "\t ... Calibration mode " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'CalibrationMethod'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->EnsensoN30
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - EnsensoN30::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	std::cout << "\t [OK] Parsing xml calibration file\n";

	return RET_OK;
}