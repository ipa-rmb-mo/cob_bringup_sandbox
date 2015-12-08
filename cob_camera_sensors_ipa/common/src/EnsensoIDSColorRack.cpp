#include <cob_vision_utils/StdAfx.h>
#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/EnsensoIDSColorRack.h"	
	#include "cob_vision_utils/GlobalDefines.h"

	#include "tinyxml.h"
	#include <fstream>
	#include <iostream>
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/EnsensoIDSColorRack.h"
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"

	#include <fstream>
#endif



using namespace ipa_CameraSensors;

#define IDS_SXGA_X_RES 1280
#define IDS_SXGA_Y_RES 1024
#define IDS_VGA_X_RES 640
#define IDS_VGA_Y_RES 480

__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr ipa_CameraSensors::CreateRangeImagingSensor_EnsensoIDSColorRack()
{
	return AbstractRangeImagingSensorPtr(new EnsensoIDSColorRack());
}

EnsensoIDSColorRack::EnsensoIDSColorRack()
	: m_ensensoCamera("EnsensoIDSColorRack_"), m_idsUEyeCamera("EnsensoIDSColorRack_")
{
	m_initialized = false;
	m_open = false;

	m_BufferSize = 1;
}

EnsensoIDSColorRack::~EnsensoIDSColorRack()
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
			std::cerr << "ERROR - EnsensoIDSColorRack::Init:" << std::endl;
			std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
		}
	}
}


unsigned long EnsensoIDSColorRack::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return ipa_Utils::RET_OK;
	}

	m_parameter_files_directory = directory;

	// Load camera parameters from xml-file
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - EnsensoIDSColorRack::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	if (m_ensensoCamera.Init(directory, 0) & RET_FAILED)
	{
		std::cerr << "ERROR - EnsensoIDSColorRack::Init:" << std::endl;
		std::cerr << "\t ... Initializing Ensenso camera failed." << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	//if (m_idsUEyeCamera.Init(directory, 0) & RET_FAILED)
	//{
	//	std::cerr << "ERROR - EnsensoIDSColorRack::Init:" << std::endl;
	//	std::cerr << "\t ... Initializing IDS color camera failed." << std::endl;
	//	return ipa_Utils::RET_FAILED;
	//}
	try
	{
		std::cout << "Opening NxLib and waiting for cameras to be detected\n";
		nxLibInitialize(true);

		// import camera calibration if file exists
		std::string calibration_file = m_parameter_files_directory + "ids_ueye_calibration.json";
		std::ifstream file(calibration_file.c_str(), std::ios::in);
		if (file.is_open() == false)
		{
			std::cerr << "WARNING - EnsensoIDSColorRack::open:" << std::endl;
			std::cerr << "\t ... Could not open calibration file '" << calibration_file << "'" << std::endl;
		}
		else
		{
			// read in file
			std::stringstream json_calibration;
			std::string line;
			while (file.eof() == false)
			{
				std::getline(file, line);
				json_calibration << line << std::endl;
			}
			m_JSONCalibration = json_calibration.str();
		}
		file.close();
	}
	catch (NxLibException ex)
	{
		std::cerr << "ERROR - EnsensoN30::Init:" << std::endl;
		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
		return RET_FAILED;
	}


	m_CameraType = ipa_CameraSensors::CAM_ENSENSO_IDS_RACK;

	// Set init flag
	m_initialized = true;

	return ipa_Utils::RET_OK;
}


unsigned long EnsensoIDSColorRack::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}
	
	if (m_ensensoCamera.Open() & RET_FAILED)
	{
		std::cerr << "ERROR - EnsensoIDSColorRack::Open:" << std::endl;
		std::cerr << "\t ... Opening Ensenso camera failed." << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	//if (m_idsUEyeCamera.Open() & RET_FAILED)
	//{
	//	std::cerr << "ERROR - EnsensoIDSColorRack::Open:" << std::endl;
	//	std::cerr << "\t ... Opening IDS color camera failed." << std::endl;
	//	return ipa_Utils::RET_FAILED;
	//}
	try
	{
		NxLibItem root; // Reference to the API tree root

		// Create an object referencing the camera's tree item, for easier access:
		m_idsUEyeCamera_nx = root[itmCameras][itmBySerialNo][m_idsUEyeSerial];
		if (!m_idsUEyeCamera_nx.exists() || (m_idsUEyeCamera_nx[itmType] != valMonocular))
		{
			std::cerr << "ERROR - EnsensoIDSColorRack::Open:" << std::endl;
			std::cerr << "\t ... Please connect an IDS uEye color camera to your computer or double check the serial entered in cameraSensors.ini." << std::endl;
			return RET_FAILED;
		}

		std::string serial = m_idsUEyeCamera_nx[itmSerialNumber].asString();
		std::cout << "Opening camera " << serial << std::endl;
		NxLibCommand open(cmdOpen); // When calling the 'execute' method in this object, it will synchronously execute the command 'cmdOpen'
		open.parameters()[itmCameras] = serial; // Set parameters for the open command
		open.execute();

		// set camera calibration
		if (m_JSONCalibration.length() > 2)
		{
			std::cout << "Setting the following camera calibration via json string:\n" << m_JSONCalibration << "\n----------" << std::endl;
			m_idsUEyeCamera_nx.setJson(m_JSONCalibration, true);
		}

		// load parameters from file
		NxLibCommand loadUEyeParameterSet(cmdLoadUEyeParameterSet);
		loadUEyeParameterSet.parameters()[itmCameras] = m_idsUEyeSerial;
		std::string filename = m_parameter_files_directory + "ids_ueye_settings.ini";
		loadUEyeParameterSet.parameters()[itmFilename] = filename;
		loadUEyeParameterSet.execute();

		m_width = m_idsUEyeCamera_nx[itmSensor][itmSize][0].asInt();
		m_height = m_idsUEyeCamera_nx[itmSensor][itmSize][1].asInt();

		//NxLibCommand loadCalibration(cmdLoadCalibration);
		//std::string cameras_str = "[\"" + m_ensensoSerial + "\",\"" + m_idsUEyeSerial + "\"]";
		//loadCalibration.parameters()[itmCameras].setJson(cameras_str, true);
		//loadCalibration.execute();
	}
	catch (NxLibException ex)
	{
		std::cerr << "ERROR - EnsensoIDSColorRack::Open:" << std::endl;
		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
		return RET_FAILED;
	}

	//t_cameraProperty prop;
	//prop.propertyID = PROP_CAMERA_RESOLUTION;
	//m_idsUEyeCamera.GetProperty(&prop);
	//m_width = prop.cameraResolution.xResolution;
	//m_height = prop.cameraResolution.yResolution;



	std::cout << "*************************************************" << std::endl;
	std::cout << "EnsensoIDSColorRack::Open: EnsensoIDSColorRack camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long EnsensoIDSColorRack::Close()
{
	if (isOpen() == false)
		return RET_OK;

	std::cout << "INFO - EnsensoIDSColorRack: Closing device..." << std::endl;

	if (m_ensensoCamera.Close() & RET_FAILED)
	{
		std::cerr << "ERROR - EnsensoIDSColorRack::Close:" << std::endl;
		std::cerr << "\t ... Closing Ensenso camera failed." << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	//if (m_idsUEyeCamera.Close() & RET_FAILED)
	//{
	//	std::cerr << "ERROR - EnsensoIDSColorRack::Close:" << std::endl;
	//	std::cerr << "\t ... Closing IDS color camera failed." << std::endl;
	//	return ipa_Utils::RET_FAILED;
	//}
	try
	{
		NxLibCommand close(cmdClose);
		close.parameters()[itmSerialNumber] = m_idsUEyeSerial;
		close.execute();
		m_idsUEyeCamera_nx = NxLibItem("");
	}
	catch (NxLibException ex)
	{
		std::cerr << "ERROR - EnsensoIDSColorRack::Close:" << std::endl;
		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
		return RET_FAILED;
	}

	m_open = false;
	return RET_OK;
}


unsigned long EnsensoIDSColorRack::SetProperty(t_cameraProperty* cameraProperty) 
{
	return ipa_Utils::RET_OK;
}


unsigned long EnsensoIDSColorRack::SetPropertyDefaults() 
{
	return ipa_Utils::RET_OK;
}


unsigned long EnsensoIDSColorRack::GetProperty(t_cameraProperty* cameraProperty) 
{
	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:	
			cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			if (isOpen())
			{
				cameraProperty->cameraResolution.xResolution = m_width;
				cameraProperty->cameraResolution.yResolution = m_height;
			}
			else
			{
				std::cout << "WARNING - EnsensoIDSColorRack::GetProperty:" << std::endl;
				std::cout << "\t ... Camera not open" << std::endl;
				std::cout << "\t ... Returning default width and height of '" << IDS_SXGA_X_RES << "' x '" << IDS_SXGA_Y_RES << "'" << std::endl;
				cameraProperty->cameraResolution.xResolution = IDS_SXGA_X_RES;
				cameraProperty->cameraResolution.yResolution = IDS_SXGA_Y_RES;
			}
			break;
		case PROP_BRIGHTNESS:	
		case PROP_WHITE_BALANCE_U:	
		case PROP_HUE:	
		case PROP_SATURATION:	
		case PROP_GAMMA:	
		case PROP_EXPOSURE_TIME:	
		case PROP_GAIN:	
		case PROP_OPTICAL_FILTER:	
		case PROP_FRAME_RATE:	
		case PROP_REGISTER:	
		case PROP_TIMEOUT:	
		default: 				
			std::cout << "ERROR - EnsensoIDSColorRack::GetProperty:" << std::endl;
			std::cout << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			return ipa_Utils::RET_FAILED;
			break;
	}

	return ipa_Utils::RET_OK;
}


unsigned long EnsensoIDSColorRack::AcquireImages(cv::Mat* rangeImage, cv::Mat* colorImage, cv::Mat* cartesianImage,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	char* rangeImageData = 0;
	char* colorImageData = 0;
	char* cartesianImageData = 0;
	int widthStepRange = -1;
	int widthStepColor = -1;
	int widthStepCartesian = -1;
	
	if(rangeImage)
	{
		// Depth image is upsampled according to the size of the color image
		rangeImage->create(m_height, m_width, CV_32FC1);
		rangeImageData = rangeImage->ptr<char>(0);
		widthStepRange = rangeImage->step;
	}
	
	if(colorImage)
	{
		colorImage->create(m_height, m_width, CV_8UC3);
		colorImageData = colorImage->ptr<char>(0);
		widthStepColor = colorImage->step;
	}	

	if(cartesianImage)
	{
		// Depth image is upsampled according to the size of the color image
		cartesianImage->create(m_height, m_width, CV_32FC3);
		cartesianImageData = cartesianImage->ptr<char>(0);
		widthStepCartesian = cartesianImage->step;
	}

	if (!rangeImage && !colorImage && !cartesianImage)
		return RET_OK;

	return AcquireImages(widthStepRange, widthStepColor, widthStepCartesian, rangeImageData, colorImageData,  cartesianImageData, getLatestFrame, undistort, grayImageType);
}

unsigned long EnsensoIDSColorRack::AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* rangeImageData, char* colorImageData, char* cartesianImageData,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	// point map z --> range image
	// point map --> cartesian image
	// ids image --> color image

	// retrieve point cloud and ids image
	try
	{
		// execute the 'Capture', 'ComputeDisparityMap' and 'RenderPointMap' commands
		std::string cameras_str = "[\"" + m_ensensoSerial + "\",\"" + m_idsUEyeSerial + "\"]";

		// grab an image
		NxLibCommand capture(cmdCapture);
		capture.parameters()[itmCameras].setJson(cameras_str, true);
		capture.execute();

		// compute the disparity map, this is the actual, computation intensive stereo matching task
		NxLibCommand computeDisparity(cmdComputeDisparityMap);
		computeDisparity.parameters()[itmCameras].setJson(cameras_str, true);
		computeDisparity.execute();
		
		NxLibItem root; // Reference to the API tree root
		root[itmParameters][itmRenderPointMap][itmTexture] = true;
		//root[itmParameters][itmRenderPointMap][itmUseOpenGL] = false;
		
		// render point map from disparity map, this converts the disparity map into XYZRGB data for each pixel from the color camera's perspective
		NxLibCommand renderPointMap(cmdRenderPointMap);
		renderPointMap.parameters()[itmCameras].setJson(cameras_str, true);
		renderPointMap.parameters()[itmCamera] = m_idsUEyeSerial;
		renderPointMap.parameters()[itmNear] = 50;
		renderPointMap.parameters()[itmFar] = 10000;
		renderPointMap.parameters()[itmFillXYCoordinates] = false;
		renderPointMap.parameters()[itmZBufferOnly] = false;
		renderPointMap.execute();

		// get info about the computed point map and copy it into a std::vector
		std::vector<float> pointMap;
		int range_width=0, range_height=0;
		root[itmImages][itmRenderPointMap].getBinaryDataInfo(&range_width, &range_height, 0,0,0,0);
		root[itmImages][itmRenderPointMap].getBinaryData(pointMap, 0);
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

		// get the color image
		std::vector<unsigned char> texture;
		int color_width=0, color_height=0;
		NxLibItem ids_camera = root[itmCameras][itmBySerialNo][m_idsUEyeSerial];
		ids_camera[itmImages][itmRectified].getBinaryDataInfo(&color_width, &color_height, 0,0,0,0);
		ids_camera[itmImages][itmRectified].getBinaryData(texture, 0);
		//root[itmImages][itmRenderPointMapTexture].getBinaryDataInfo(&color_width, &color_height, 0,0,0,0);
		//root[itmImages][itmRenderPointMapTexture].getBinaryData(texture, 0);
		if (colorImageData)
		{
			const int number_elements = color_width*color_height;
			unsigned char* p_colorImageData = (unsigned char*)colorImageData;
			for (int i=0; i<number_elements; ++i)
			{
				*p_colorImageData = texture[3*i+2];
				++p_colorImageData;
				*p_colorImageData = texture[3*i+1];
				++p_colorImageData;
				*p_colorImageData = texture[3*i];
				++p_colorImageData;
			}
		}
	}
	catch (NxLibException ex)
	{
		std::cerr << "ERROR - EnsensoIDSColorRack::AcquireImages:" << std::endl;
		std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
	}

	return  RET_OK;
}


unsigned long EnsensoIDSColorRack::SaveParameters(const char* filename) 
{
	return ipa_Utils::RET_OK;
}


unsigned long EnsensoIDSColorRack::LoadParameters(const char* filename, int cameraIndex)
{
	// Load EnsensoIDSColorRack parameters.
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - EnsensoIDSColorRack::LoadParameters:" << std::endl;
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
//	BEGIN LibCameraSensors->EnsensoIDSColorRack
//************************************************************************************
			// Tag element "EnsensoIDSColorRack of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_Ensenso = NULL;
			std::stringstream ss;
			ss << "EnsensoIDSColorRack_" << cameraIndex;
			p_xmlElement_Root_Ensenso = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_Ensenso )
			{
//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoIDSColorRack->EnsensoSerial
//************************************************************************************
				// Subtag element "EnsensoSerial" of XML Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Ensenso->FirstChildElement( "EnsensoSerial" );
				if ( p_xmlElement_Child )
				{
					m_ensensoSerial = "";
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'EnsensoSerial'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					m_ensensoSerial = tempString;
				}
				else
				{
					std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'EnsensoSerial'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoIDSColorRack->IDSSerial
//************************************************************************************
				// Subtag element "IDSSerial" of XML Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Ensenso->FirstChildElement( "IDSSerial" );
				if ( p_xmlElement_Child )
				{
					m_idsUEyeSerial = "";
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'IDSSerial'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					m_idsUEyeSerial = tempString;
				}
				else
				{
					std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'IDSSerial'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoIDSColorRack->Role
//************************************************************************************
				// Subtag element "Role" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Ensenso->FirstChildElement( "Role" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Role'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					if (tempString == "MASTER") m_RangeCameraParameters.m_CameraRole = MASTER;
					else if (tempString == "SLAVE") m_RangeCameraParameters.m_CameraRole = SLAVE;
					else
					{
						std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
						std::cerr << "\t ... Role " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Role'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoIDSColorRack->VideoFormat
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
						std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
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
						std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
						std::cerr << "\t ... Video format " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'VideoFormat'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->EnsensoIDSColorRack->CalibrationMethod
//************************************************************************************
				// Subtag element "OperationMode" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Ensenso->FirstChildElement( "CalibrationMethod" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "name", &tempString ) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
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
						std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
						std::cerr << "\t ... Calibration mode " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'CalibrationMethod'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->EnsensoIDSColorRack
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - EnsensoIDSColorRack::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	std::cout << "\t [OK] Parsing xml calibration file\n";

	return RET_OK;
}