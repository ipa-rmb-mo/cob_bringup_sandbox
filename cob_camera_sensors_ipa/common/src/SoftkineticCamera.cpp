/****************************************************************
*
* Copyright (c) 2016
*
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Project name: care-o-bot
* ROS stack name: cob_bringup_sandbox
* ROS package name: cob_camera_sensors_ipa
* Description: Interface for Softkinetic cameras.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
* Supervised by: 
*
* Date of creation: October 2016
* ToDo:
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

#include <cob_vision_utils/StdAfx.h>

#ifdef __BUILD_WITH_SOFTKINETIC__

#ifdef __LINUX__
	#include "cob_camera_sensors_ipa/SoftkineticCamera.h"	
	#include "cob_vision_utils/GlobalDefines.h"
	#include "tinyxml.h"
	#include <iostream>
#else
	#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/SoftkineticCamera.h"
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
	#include <functional>
#endif

#include <fstream>

using namespace ipa_CameraSensors;

// Macro to disable unused parameter compiler warnings
#define UNUSED(x) (void)(x)

__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr ipa_CameraSensors::CreateRangeImagingSensor_SoftkineticCamera()
{
	return AbstractRangeImagingSensorPtr(new SoftkineticCamera());
}

SoftkineticCamera::SoftkineticCamera()
	: m_confidence_threshold(100), m_device_index(0), m_device_serial_number("")
{
	m_initialized = false;
	m_open = false;

	m_BufferSize = 1;



}

SoftkineticCamera::~SoftkineticCamera()
{
	if (isOpen())
		Close();

	if (isInitialized())
	{

	}
}


unsigned long SoftkineticCamera::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return ipa_Utils::RET_OK;
	}

	m_CameraType = ipa_CameraSensors::CAM_SOFTKINETIC;
	m_parameter_files_directory = directory;

	// Load camera parameters from xml-file
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - SoftkineticCamera::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	
	// Initialize the SDK
	m_context = DepthSense::Context::create();
	Callback<void(DepthSense::Context,DepthSense::Context::DeviceAddedData)>::func = std::bind(&SoftkineticCamera::OnDeviceConnected, this, std::placeholders::_1, std::placeholders::_2);
	callback_deviceadd_t func_deviceadd = static_cast<callback_deviceadd_t>(Callback<void(DepthSense::Context,DepthSense::Context::DeviceAddedData)>::callback);
	m_context.deviceAddedEvent().connect(func_deviceadd);
	Callback<void(DepthSense::Context,DepthSense::Context::DeviceRemovedData)>::func = std::bind(&SoftkineticCamera::OnDeviceRemoved, this, std::placeholders::_1, std::placeholders::_2);
	callback_deviceremove_t func_deviceremove = static_cast<callback_deviceremove_t>(Callback<void(DepthSense::Context,DepthSense::Context::DeviceRemovedData)>::callback);
	m_context.deviceRemovedEvent().connect(func_deviceremove);

	// Set init flag
	m_initialized = true;

	return ipa_Utils::RET_OK;
}


unsigned long SoftkineticCamera::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}
	

	//t_cameraProperty prop;
	//prop.propertyID = PROP_CAMERA_RESOLUTION;
	//m_idsUEyeCamera.GetProperty(&prop);
	//m_width = prop.cameraResolution.xResolution;
	//m_height = prop.cameraResolution.yResolution;



	std::cout << "*************************************************" << std::endl;
	std::cout << "SoftkineticCamera::Open: SoftkineticCamera camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long SoftkineticCamera::Close()
{
	if (isOpen() == false)
		return RET_OK;

	std::cout << "INFO - SoftkineticCamera: Closing device..." << std::endl;

	m_open = false;
	return RET_OK;
}


void SoftkineticCamera::OnDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data)
{
    UNUSED(context);
    UNUSED(data);
}

void SoftkineticCamera::OnDeviceRemoved(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data)
{
    UNUSED(context);
    UNUSED(data);
}


unsigned long SoftkineticCamera::SetProperty(t_cameraProperty* cameraProperty) 
{
	return ipa_Utils::RET_OK;
}


unsigned long SoftkineticCamera::SetPropertyDefaults() 
{
	return ipa_Utils::RET_OK;
}


unsigned long SoftkineticCamera::GetProperty(t_cameraProperty* cameraProperty) 
{
	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:	
			//cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			//if (isOpen())
			//{
			//	cameraProperty->cameraResolution.xResolution = m_width;
			//	cameraProperty->cameraResolution.yResolution = m_height;
			//}
			//else
			//{
			//	std::cout << "WARNING - SoftkineticCamera::GetProperty:" << std::endl;
			//	std::cout << "\t ... Camera not open" << std::endl;
			//	std::cout << "\t ... Returning default width and height of '" << IDS_SXGA_X_RES << "' x '" << IDS_SXGA_Y_RES << "'" << std::endl;
			//	cameraProperty->cameraResolution.xResolution = IDS_SXGA_X_RES;
			//	cameraProperty->cameraResolution.yResolution = IDS_SXGA_Y_RES;
			//}
			//break;
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
			std::cout << "ERROR - SoftkineticCamera::GetProperty:" << std::endl;
			std::cout << "\t ... Property " << cameraProperty->propertyID << " unspecified.";
			return ipa_Utils::RET_FAILED;
			break;
	}

	return ipa_Utils::RET_OK;
}


unsigned long SoftkineticCamera::AcquireImages(cv::Mat* rangeImage, cv::Mat* colorImage, cv::Mat* cartesianImage,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	char* rangeImageData = 0;
	char* colorImageData = 0;
	char* cartesianImageData = 0;
	int widthStepRange = -1;
	int widthStepColor = -1;
	int widthStepCartesian = -1;
	
	//if(rangeImage)
	//{
	//	// Depth image is upsampled according to the size of the color image
	//	rangeImage->create(m_height, m_width, CV_32FC1);
	//	rangeImageData = rangeImage->ptr<char>(0);
	//	widthStepRange = rangeImage->step;
	//}
	//
	//if(colorImage)
	//{
	//	colorImage->create(m_height, m_width, CV_8UC3);
	//	colorImageData = colorImage->ptr<char>(0);
	//	widthStepColor = colorImage->step;
	//}	

	//if(cartesianImage)
	//{
	//	// Depth image is upsampled according to the size of the color image
	//	cartesianImage->create(m_height, m_width, CV_32FC3);
	//	cartesianImageData = cartesianImage->ptr<char>(0);
	//	widthStepCartesian = cartesianImage->step;
	//}

	if (!rangeImage && !colorImage && !cartesianImage)
		return RET_OK;

	return AcquireImages(widthStepRange, widthStepColor, widthStepCartesian, rangeImageData, colorImageData,  cartesianImageData, getLatestFrame, undistort, grayImageType);
}

unsigned long SoftkineticCamera::AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* rangeImageData, char* colorImageData, char* cartesianImageData,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	// point map z --> range image
	// point map --> cartesian image
	// ids image --> color image

	// retrieve point cloud and ids image
	//try
	//{
	//	// execute the 'Capture', 'ComputeDisparityMap' and 'RenderPointMap' commands
	//	std::string cameras_str = "[\"" + m_ensensoSerial + "\",\"" + m_idsUEyeSerial + "\"]";

	//	// grab an image
	//	NxLibCommand capture(cmdCapture);
	//	capture.parameters()[itmCameras].setJson(cameras_str, true);
	//	capture.execute();

	//	// compute the disparity map, this is the actual, computation intensive stereo matching task
	//	NxLibCommand computeDisparity(cmdComputeDisparityMap);
	//	computeDisparity.parameters()[itmCameras].setJson(cameras_str, true);
	//	computeDisparity.execute();
	//	
	//	NxLibItem root; // Reference to the API tree root
	//	root[itmParameters][itmRenderPointMap][itmTexture] = true;
	//	//root[itmParameters][itmRenderPointMap][itmUseOpenGL] = false;
	//	
	//	// render point map from disparity map, this converts the disparity map into XYZRGB data for each pixel from the color camera's perspective
	//	NxLibCommand renderPointMap(cmdRenderPointMap);
	//	renderPointMap.parameters()[itmCameras].setJson(cameras_str, true);
	//	renderPointMap.parameters()[itmCamera] = m_idsUEyeSerial;
	//	renderPointMap.parameters()[itmNear] = 50;
	//	renderPointMap.parameters()[itmFar] = 10000;
	//	renderPointMap.parameters()[itmFillXYCoordinates] = false;
	//	renderPointMap.parameters()[itmZBufferOnly] = false;
	//	renderPointMap.execute();

	//	// get info about the computed point map and copy it into a std::vector
	//	std::vector<float> pointMap;
	//	int range_width=0, range_height=0;
	//	root[itmImages][itmRenderPointMap].getBinaryDataInfo(&range_width, &range_height, 0,0,0,0);
	//	root[itmImages][itmRenderPointMap].getBinaryData(pointMap, 0);
	//	if (cartesianImageData)
	//	{
	//		const int number_elements = range_height*range_width;
	//		float* p_cartesianImageData = (float*)cartesianImageData;
	//		for (int i=0; i<number_elements; ++i)
	//		{
	//			*p_cartesianImageData = 0.001f * pointMap[3*i];
	//			++p_cartesianImageData;
	//			*p_cartesianImageData = 0.001f * pointMap[3*i+1];
	//			++p_cartesianImageData;
	//			*p_cartesianImageData = 0.001f * pointMap[3*i+2];
	//			++p_cartesianImageData;
	//		}
	//	}
	//	if (rangeImageData)
	//	{
	//		const int number_elements = range_height*range_width;
	//		float* p_rangeImageData = (float*)rangeImageData;
	//		for (int i=0; i<number_elements; ++i)
	//		{
	//			*p_rangeImageData = 0.001f * pointMap[3*i+2];
	//			++p_rangeImageData;
	//		}
	//	}

	//	// get the color image
	//	std::vector<unsigned char> texture;
	//	int color_width=0, color_height=0;
	//	NxLibItem ids_camera = root[itmCameras][itmBySerialNo][m_idsUEyeSerial];
	//	ids_camera[itmImages][itmRectified].getBinaryDataInfo(&color_width, &color_height, 0,0,0,0);
	//	ids_camera[itmImages][itmRectified].getBinaryData(texture, 0);
	//	//root[itmImages][itmRenderPointMapTexture].getBinaryDataInfo(&color_width, &color_height, 0,0,0,0);
	//	//root[itmImages][itmRenderPointMapTexture].getBinaryData(texture, 0);
	//	if (colorImageData)
	//	{
	//		const int number_elements = color_width*color_height;
	//		unsigned char* p_colorImageData = (unsigned char*)colorImageData;
	//		for (int i=0; i<number_elements; ++i)
	//		{
	//			*p_colorImageData = texture[3*i+2];
	//			++p_colorImageData;
	//			*p_colorImageData = texture[3*i+1];
	//			++p_colorImageData;
	//			*p_colorImageData = texture[3*i];
	//			++p_colorImageData;
	//		}
	//	}
	//}
	//catch (NxLibException ex)
	//{
	//	std::cerr << "ERROR - SoftkineticCamera::AcquireImages:" << std::endl;
	//	std::cerr << ex.getItemPath() << " has error " << ex.getErrorCode() << ": " << ex.getErrorText() << std::endl;
	//}

	return  RET_OK;
}


unsigned long SoftkineticCamera::SaveParameters(const char* filename) 
{
	return ipa_Utils::RET_OK;
}


unsigned long SoftkineticCamera::LoadParameters(const char* filename, int cameraIndex)
{
	// Load SoftkineticCamera parameters.
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - SoftkineticCamera::LoadParameters:" << std::endl;
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
//	BEGIN LibCameraSensors->SoftkineticCamera
//************************************************************************************
			// Tag element "SoftkineticCamera of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_Softkinetic = NULL;
			std::stringstream ss;
			ss << "SoftkineticCamera_" << cameraIndex;
			p_xmlElement_Root_Softkinetic = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_Softkinetic )
			{
				std::string element_name;
				std::string attribute_name;
				std::string tempString;
//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "Role";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					if (tempString == "MASTER") m_RangeCameraParameters.m_CameraRole = MASTER;
					else if (tempString == "SLAVE") m_RangeCameraParameters.m_CameraRole = SLAVE;
					else
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Role " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "CalibrationMethod";
				attribute_name = "name";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
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
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Calibration mode " << tempString << " unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}


//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "DeviceIndex";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					std::stringstream ss(tempString);
					ss >> m_device_index;
					if (m_device_index < 0)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... The " << element_name << " cannot be less than 0." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "DeviceSerialNumber";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &m_device_serial_number) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					if (m_device_serial_number == "")
					{
						std::cerr << "INFO - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Using DeviceIndex to select camera." << std::endl;
					}
					else
					{
						std::cerr << "INFO - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Using " << element_name << " '" << m_device_serial_number << "' to select camera." << std::endl;
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "ConfidenceThreshold";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					std::stringstream ss(tempString);
					ss >> m_confidence_threshold;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}


////////////////////////////////////////////////////////////////////////////////
/////       Get the configuration parameters for the color camera          /////
////////////////////////////////////////////////////////////////////////////////

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "VideoFormatRGB";
				attribute_name = "type";
				m_color_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					// Get the resolution of the RGB camera
					if (tempString == "QQVGA" || tempString == "qqvga")
					{
						m_color_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_QQVGA;
					}
					else if (tempString == "QVGA" || tempString == "qvga")
					{
						m_color_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
					}
					else if (tempString == "VGA" || tempString == "vga")
					{
						m_color_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
					}
					else if (tempString == "NHD" || tempString == "nhd")
					{
						m_color_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_NHD;
					}
					else if (tempString == "WXGA_H" || tempString == "wxga_h")
					{
						m_color_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_WXGA_H;
					}
					else
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Video format RGB '" << tempString << "' unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "FramerateRGB";
				attribute_name = "value";
				m_color_config.base_config.framerate = 30;
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					std::stringstream ss(tempString);
					ss >> m_color_config.base_config.framerate;
					if (m_color_config.base_config.framerate < 0)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Framerate RGB '" << m_color_config.base_config.framerate << "' unspecified." << std::endl;
						return (RET_FAILED);
					}
					if (m_color_config.base_config.framerate == 0)
					{
						std::cerr << "WARNING - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Framerate RGB = 0, the camera will be disabled." << std::endl;
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "CompressionRGB";
				attribute_name = "value";
				m_color_config.base_config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the compression mode of the RGB camera
					if (tempString == "MJPEG" || tempString == "mjpeg")
					{
						m_color_config.base_config.compression = DepthSense::COMPRESSION_TYPE_MJPEG;
					}
					else if (tempString == "YUY2" || tempString == "yuy2")
					{
						m_color_config.base_config.compression = DepthSense::COMPRESSION_TYPE_YUY2;
					}
					else
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... " << element_name << " '" << tempString << "' unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "PowerLineFrequency";
				attribute_name = "value";
				m_color_config.base_config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_60HZ;
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the powerline frequency (0 is disabled)
					std::stringstream ss(tempString);
					int32_t powerline_hertz = 60;
					ss >> powerline_hertz;
					if (powerline_hertz == 60)
					{
						m_color_config.base_config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_60HZ;
					}
					else if (powerline_hertz == 50)
					{
						m_color_config.base_config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_50HZ;
					}
					else if (powerline_hertz == 0)
					{
						m_color_config.base_config.powerLineFrequency = DepthSense::POWER_LINE_FREQUENCY_DISABLED;
						std::cerr << "WARNING - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... " << element_name << " disabled." << std::endl;
					}
					else
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Invalid powerline frequency '" << powerline_hertz << "' Hz." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "BrightnessRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the brightness of the RGB camera
					std::stringstream ss(tempString);
					ss >> m_color_config.brightness;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "ContrastRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the contrast of the RGB camera
					std::stringstream ss(tempString);
					ss >> m_color_config.contrast;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "SaturationRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the saturation of the RGB camera
					std::stringstream ss(tempString);
					ss >> m_color_config.saturation;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "HueRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the hue of the RGB camera
					std::stringstream ss(tempString);
					ss >> m_color_config.hue;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "GammaRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the gamma of the RGB camera
					std::stringstream ss(tempString);
					ss >> m_color_config.gamma;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "SharpnessRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the sharpness of the RGB camera
					std::stringstream ss(tempString);
					ss >> m_color_config.sharpness;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "ExposureRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the exposure of the RGB camera (0 means autoexposure)
					std::stringstream ss(tempString);
					ss >> m_color_config.exposure;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "WhiteBalanceRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the white balance of the RGB camera (0 means automatic white balance)
					std::stringstream ss(tempString);
					ss >> m_color_config.white_balance;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "EnableCompressionRGB";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the enable/disable for compressed data from the camera
					std::stringstream ss(tempString);
					int value;
					ss >> value;
					if (value == 1)
						m_color_config.enable_compressed_data = true;
					else
						m_color_config.enable_compressed_data = false;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

////////////////////////////////////////////////////////////////////////////////
/////       Set the configuration parameters for the color camera          /////
////////////////////////////////////////////////////////////////////////////////
				// Set the rgb camera enabled/disabled
				if (m_color_config.base_config.framerate > 0)
				{
					m_color_config.enable_color_map = true;
				}
				else
				{
					m_color_config.enable_color_map = false;
				}
				// Set the exposure controls
				if (m_color_config.exposure == 0)
				{
					m_color_config.enable_priority_auto_exposure_mode = true;
					m_color_config.auto_exposure_mode = DepthSense::EXPOSURE_AUTO_APERTURE_PRIORITY;
				}
				else
				{
					m_color_config.enable_priority_auto_exposure_mode = false;
					m_color_config.auto_exposure_mode = DepthSense::EXPOSURE_AUTO_MANUAL;
				}
				// Set the white balance controls
				if (m_color_config.white_balance == 0)
				{
					m_color_config.enable_auto_white_balance = true;
				}
				else
				{
					m_color_config.enable_auto_white_balance = false;
				}

////////////////////////////////////////////////////////////////////////////////
/////       Get the configuration parameters for the depth camera          /////
////////////////////////////////////////////////////////////////////////////////

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "VideoFormatDepth";
				attribute_name = "type";
				m_depth_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}

					// Get the resolution of the depth camera
					if (tempString == "QQVGA" || tempString == "qqvga")
					{
						m_depth_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_QQVGA;
					}
					else if (tempString == "QVGA" || tempString == "qvga")
					{
						m_depth_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_QVGA;
					}
					else if (tempString == "VGA" || tempString == "vga")
					{
						m_depth_config.base_config.frameFormat = DepthSense::FRAME_FORMAT_VGA;
					}
					else
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... " << element_name << " '" << tempString << "' unspecified." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "FramerateDepth";
				attribute_name = "value";
				m_depth_config.base_config.framerate = 30;
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					std::stringstream ss(tempString);
					ss >> m_depth_config.base_config.framerate;
					if (m_depth_config.base_config.framerate < 0)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... " << element_name << " '" << m_depth_config.base_config.framerate << "' unspecified." << std::endl;
						return (RET_FAILED);
					}
					if (m_depth_config.base_config.framerate == 0)
					{
						std::cerr << "WARNING - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... " << element_name << " = 0, the camera will be disabled." << std::endl;
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "CameraModeDepth";
				attribute_name = "value";
				m_depth_config.base_config.mode = DepthSense::DepthNode::CAMERA_MODE_LONG_RANGE;
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the operating mode of the depth camera
					if (tempString == "LONG" || tempString == "long")
					{
						m_depth_config.base_config.mode = DepthSense::DepthNode::CAMERA_MODE_LONG_RANGE;
					}
					else if (tempString == "CLOSE" || tempString == "close")
					{
						m_depth_config.base_config.mode = DepthSense::DepthNode::CAMERA_MODE_CLOSE_MODE;
					}
					else
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Invalid depth camera mode '" << m_depth_config.base_config.mode << "'." << std::endl;
						return (RET_FAILED);
					}
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "SaturationDepth";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the saturation of the RGB camera
					std::stringstream ss(tempString);
					int value;
					ss >> value;
					if (value == 1)
						m_depth_config.base_config.saturation = true;
					else
						m_depth_config.base_config.saturation = false;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

//************************************************************************************
//	BEGIN LibCameraSensors->SoftkineticCamera->element_name
				element_name = "IlluminationLevelDepth";
				attribute_name = "value";
//************************************************************************************
				// Subtag element "element_name" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_Softkinetic->FirstChildElement(element_name);
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute(attribute_name, &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute '" << attribute_name << "' of tag '" << element_name << "'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					// Get the illumination level for the depth camera
					std::stringstream ss(tempString);
					ss >> m_depth_config.illumination_level;
				}
				else
				{
					std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag '" << element_name << "'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}

////////////////////////////////////////////////////////////////////////////////
/////       Set the configuration parameters for the depth camera          /////
////////////////////////////////////////////////////////////////////////////////
				// Set the depth camera enabled/disabled
				if (m_depth_config.base_config.framerate > 0)
				{
					m_depth_config.enable_floatingpoint_vertices = true;
					m_depth_config.enable_uv_map = true;
					m_depth_config.enable_xyz_depth = false;
					m_depth_config.enable_floatingpoint_xyz_depth = false;
					m_depth_config.enable_accelerometer = false;
					m_depth_config.enable_confidence_map = true;
					m_depth_config.enable_depth_map = false;
					m_depth_config.enable_floatingpoint_depth_map = true;
					m_depth_config.enable_phase_map = false;
					m_depth_config.enable_vertices = false;
				}
				else
				{
					m_depth_config.enable_floatingpoint_vertices = false;
					m_depth_config.enable_uv_map = false;
					m_depth_config.enable_xyz_depth = false;
					m_depth_config.enable_floatingpoint_xyz_depth = false;
					m_depth_config.enable_accelerometer = false;
					m_depth_config.enable_confidence_map = false;
					m_depth_config.enable_depth_map = false;
					m_depth_config.enable_floatingpoint_depth_map = false;
					m_depth_config.enable_phase_map = false;
					m_depth_config.enable_vertices = false;
				}
			}

//************************************************************************************
//	END LibCameraSensors->SoftkineticCamera
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - SoftkineticCamera::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	std::cout << "\t [OK] Parsing xml calibration file\n";

	return RET_OK;
}

#endif // #ifdef __BUILD_WITH_SOFTKINETIC__