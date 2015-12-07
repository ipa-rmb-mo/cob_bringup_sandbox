/****************************************************************
*
* Copyright (c) 2010
*
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA)
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Project name: care-o-bot
* ROS stack name: cob_driver
* ROS package name: cob_camera_sensors
* Description: Interface for IDS uEye color cameras.
*
* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*
* Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
* Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
*
* Date of creation: November 2015
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
#ifdef __LINUX__
#include "cob_camera_sensors_ipa/IDSuEyeCamera.h"

#include "tinyxml/tinyxml.h"
#include <iostream>
#else
#include "cob_bringup_sandbox/cob_camera_sensors_ipa/common/include/cob_camera_sensors_ipa/IDSuEyeCamera.h"

#endif


using namespace ipa_CameraSensors;

__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr ipa_CameraSensors::CreateColorCamera_IDSuEyeCamera()
{
	return AbstractColorCameraPtr(new IDSuEyeCamera());
}


IDSuEyeCamera::IDSuEyeCamera(const std::string xmlTagName)
	: m_xmlTagName(xmlTagName)
{
	m_initialized = false;
	m_open = false;

	m_pcImageMemoryId = 0;
	m_pcImageMemory = 0;
	m_BufferSize = 1;
	m_cameraDevice = 0;
	m_width=0;
	m_height=0;
}


IDSuEyeCamera::~IDSuEyeCamera()
{
	Close();

	if (m_cameraDevice != 0)
	{
		is_ExitCamera(m_cameraDevice);
		m_cameraDevice = 0;
		m_initialized = false;
	}
}


unsigned long IDSuEyeCamera::Init(std::string directory, int cameraIndex)
{
	if (isInitialized())
	{
		return (RET_OK | RET_CAMERA_ALREADY_INITIALIZED);
	}

	// load parameters
	if (LoadParameters((directory + "cameraSensorsIni.xml").c_str(), cameraIndex) & RET_FAILED)
	{
		std::cerr << "ERROR - IDSuEyeCamera::Init:" << std::endl;
		std::cerr << "\t ... Parsing xml configuration file failed." << std::endl;
		return RET_FAILED;
	}

	m_cameraDevice = (HIDS)(cameraIndex+1);
	INT nRet = is_InitCamera(&m_cameraDevice, 0);	
	/************************************************************************************************/
	/*  If the camera returns with "IS_STARTER_FW_UPLOAD_NEEDED", an upload of a new firmware       */
	/*  is necessary. This upload can take several seconds. We recommend to check the required      */
	/*  time with the function is_GetDuration().                                                    */
	/*                                                                                              */
	/*  In this case, the camera can only be opened if the flag "IS_ALLOW_STARTER_FW_UPLOAD"        */ 
	/*  is "OR"-ed to m_hCam. This flag allows an automatic upload of the firmware.                 */                        
	/************************************************************************************************/
	if (nRet == IS_STARTER_FW_UPLOAD_NEEDED)
	{
		// Time for the firmware upload = 25 seconds by default
		INT nUploadTime = 25000;
		is_GetDuration(m_cameraDevice, IS_STARTER_FW_UPLOAD, &nUploadTime);

		std::cerr << "WARNING - IDSuEyeCamera::Init:" << std::endl;
		std::cerr << "\t ... This camera requires a new firmware. The upload will take about " << nUploadTime * 0.001 << "seconds. Please wait and keep camera and computer powered ..." << std::endl;
	
		// Try again to open the camera. This time we allow the automatic upload of the firmware by
		// specifying "IS_ALLOW_STARTER_FIRMWARE_UPLOAD"
		m_cameraDevice = (HIDS)(((INT)m_cameraDevice) | IS_ALLOW_STARTER_FW_UPLOAD); 
		nRet = is_InitCamera(&m_cameraDevice, 0);   
	}
	if(nRet != IS_SUCCESS)
	{
		std::cerr << "ERROR - IDSuEyeCamera::Init:" << std::endl;
		std::cerr << "\t ... Camera initialization failed." << std::endl;
		return RET_FAILED;
	}

	// set camera type
	m_CameraType = ipa_CameraSensors::CAM_IDS_UEYE;

	// Set init flag
	m_initialized = true;

	return RET_OK;
}


unsigned long IDSuEyeCamera::Open()
{
	if (!isInitialized())
	{
		return (RET_FAILED | RET_CAMERA_NOT_INITIALIZED);
	}

	if (isOpen())
	{
		return (RET_OK | RET_CAMERA_ALREADY_OPEN);
	}

	std::cout << "INFO - IDSuEyeCamera::Open():" << std::endl;
	std::cout << "\t ... Opening camera device" << std::endl;
	std::cout << "\t ... This may take some seconds" << std::endl;

	// get sensor info
	SENSORINFO sensorInfo;
	is_GetSensorInfo(m_cameraDevice, &sensorInfo);
	m_width = sensorInfo.nMaxWidth;
	m_height = sensorInfo.nMaxHeight;

	// set data format of images
	is_SetColorMode(m_cameraDevice, IS_CM_BGR8_PACKED);

	// memory initialization
	INT success = is_AllocImageMem(m_cameraDevice, m_width, m_height, 24, &m_pcImageMemory, &m_pcImageMemoryId);
	if (success != IS_SUCCESS)
	{
		std::cerr << "ERROR - IDSuEyeCamera::Open:" << std::endl;
		std::cerr << "\t ... Memory allocation failed." << std::endl;
		return RET_FAILED;
	}
	success = is_SetImageMem(m_cameraDevice, m_pcImageMemory, m_pcImageMemoryId);	// set memory active
	if (success != IS_SUCCESS)
	{
		std::cerr << "ERROR - IDSuEyeCamera::Open:" << std::endl;
		std::cerr << "\t ... Image memory activation failed." << std::endl;
		return RET_FAILED;
	}

	// display initialization
	IS_SIZE_2D imageSize;
	imageSize.s32Width = m_width;
	imageSize.s32Height = m_height;
	is_AOI(m_cameraDevice, IS_AOI_IMAGE_SET_SIZE, (void*)&imageSize, sizeof(imageSize));

	// set display mode to bitmap
	is_SetDisplayMode(m_cameraDevice, IS_SET_DM_DIB);

	// camera remains ready for capture and immediately captures and transfers an image upon calling 'is_FreezeVideo()'
	is_SetExternalTrigger(m_cameraDevice, IS_SET_TRIGGER_SOFTWARE);

	std::cout << "*************************************************" << std::endl;
	std::cout << "IDSuEyeCamera::Open: IDS uEye camera device OPEN" << std::endl;
	std::cout << "*************************************************" << std::endl << std::endl;
	m_open = true;

	return RET_OK;
}


unsigned long IDSuEyeCamera::Close()
{
	if (m_open == true)
	{
		if (m_cameraDevice != 0)
		{
			//free old image memory
			is_FreeImageMem(m_cameraDevice, m_pcImageMemory, m_pcImageMemoryId);
		}
		m_open = false;
	}

	return RET_OK;
}


unsigned long IDSuEyeCamera::GetColorImage(char* colorImageData, bool getLatestFrame)
{
	if (!isOpen())
	{
		std::cerr << "ERROR - IDSuEyeCamera::GetColorImage:" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	// acquire image
	if(is_FreezeVideo(m_cameraDevice, IS_WAIT) != IS_SUCCESS)
	{
		std::cerr << "ERROR - IDSuEyeCamera::GetColorImage:" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return RET_FAILED;
	}

	is_CopyImageMem(m_cameraDevice, m_pcImageMemory, m_pcImageMemoryId, colorImageData);

	return RET_OK;
}

unsigned long IDSuEyeCamera::GetColorImage(cv::Mat* colorImage, bool getLatestFrame)
{
	if (!isOpen())
	{
		std::cerr << "ERROR - IDSuEyeCamera::GetColorImage:" << std::endl;
		std::cerr << "\t ... Color camera not open." << std::endl;
		return (RET_FAILED | RET_CAMERA_NOT_OPEN);
	}

	colorImage->create(m_height, m_width, CV_8UC3);

	return GetColorImage((char*)colorImage->ptr(0), true);
}


unsigned long IDSuEyeCamera::GetProperty(t_cameraProperty* cameraProperty)
{
	int ret = 0;

	switch (cameraProperty->propertyID)
	{
		case PROP_CAMERA_RESOLUTION:	
			cameraProperty->cameraResolution.xResolution = m_width;
			cameraProperty->cameraResolution.yResolution = m_height;
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
			std::cout << "IDSuEyeCamera::GetProperty: Property " << cameraProperty->propertyID << " unspecified.";
			ret = -1; 
			break;
	}

	if (ret < 0)
	{
		return RET_FAILED;
	}
	else
	{
		return RET_OK;
	}
}


unsigned long IDSuEyeCamera::TestCamera(const char* filename) 
{
	if (AbstractColorCamera::TestCamera(filename) & RET_FAILED)
	{
		return RET_FAILED;
	}
	return RET_OK;
}


unsigned long IDSuEyeCamera::LoadParameters(const char* filename, int cameraIndex)
{
	// Load IDSuEyeCamera parameters.
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( filename ));
	
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - IDSuEyeCamera::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):\n";
		std::cerr << "\t ... '" << filename << "'" << std::endl;
		return (RET_FAILED | RET_FAILED_OPEN_FILE);
	}
	std::cout << "INFO - IDSuEyeCamera::LoadParameters:" << std::endl;
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
//	BEGIN LibCameraSensors->IDSuEyeCamera
//************************************************************************************
			// Tag element "IDSuEyeCamera of Xml Inifile		
			TiXmlElement *p_xmlElement_Root_OCVC = NULL;
			std::stringstream ss;
			ss << m_xmlTagName << cameraIndex;
			p_xmlElement_Root_OCVC = p_xmlElement_Root->FirstChildElement( ss.str() );
			if ( p_xmlElement_Root_OCVC )
			{
				
//************************************************************************************
//	BEGIN LibCameraSensors->AVTPikeCam->Resolution
//************************************************************************************
				// Subtag element "XSize" of Xml Inifile
				TiXmlElement* p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OCVC->FirstChildElement( "Resolution" );
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute("width", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - IDSuEyeCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'width' of tag 'Resolution'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_ImageWidth.str(" ");	// Clear stringstream
						m_ColorCameraParameters.m_ImageWidth.clear();		// Reset flags
						m_ColorCameraParameters.m_ImageWidth << tempString;
					}
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute("height", &tempString) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - IDSuEyeCamera::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'height' of tag 'Resolution'." << std::endl;
						return (RET_FAILED | RET_XML_ATTR_NOT_FOUND);
					}
					else
					{
						m_ColorCameraParameters.m_ImageHeight.str( " " );	// Clear stringstream
						m_ColorCameraParameters.m_ImageHeight.clear();		// Reset flags
						m_ColorCameraParameters.m_ImageHeight << tempString;
					}
				}
				else
				{
					std::cerr << "ERROR - IDSuEyeCamera::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Resolution'." << std::endl;
					return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
				}
			}

//************************************************************************************
//	END LibCameraSensors->IDSuEyeCamera
//************************************************************************************
			else 
			{
				std::cerr << "ERROR - IDSuEyeCamera::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag '" << ss.str() << "'" << std::endl;
				return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
			}
		}

//************************************************************************************
//	END LibCameraSensors
//************************************************************************************
		else 
		{
			std::cerr << "ERROR - IDSuEyeCamera::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'LibCameraSensors'." << std::endl;
			return (RET_FAILED | RET_XML_TAG_NOT_FOUND);
		}
	}
	
	std::cout << "\t [OK] Parsing xml calibration file\n";

	return RET_OK;
}