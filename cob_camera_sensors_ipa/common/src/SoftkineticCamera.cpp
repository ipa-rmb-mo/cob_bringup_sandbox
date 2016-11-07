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
		// nothing to clean
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
	
	////////////////////////////////////////////////////////////////////////////////
	// Get the currently available devices
	std::vector<DepthSense::Device> devices = m_context.getDevices();
	if (devices.size() == 0)
	{
		std::cerr << "ERROR - SoftkineticCamera::Open:" << std::endl;
		std::cerr << "\t ... No device connected." << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	if (!((size_t)m_device_index < devices.size()))
	{
		std::cerr << "ERROR - SoftkineticCamera::Open:" << std::endl;
		std::cerr << "\t ... Device index " << m_device_index << " is invalid for " << devices.size() << " devices." << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	int32_t real_device_index = 0;
	if (m_device_serial_number == "")
	{
		real_device_index = m_device_index;
	}
	else
	{
		real_device_index = -1;
		for (int32_t idx = 0; idx < (int32_t)devices.size(); idx++)
		{
			DepthSense::Device& current_device = devices[idx];
			if (current_device.getSerialNumber() == m_device_serial_number)
			{
				real_device_index = idx;
				std::cerr << "INFO - SoftkineticCamera::Open:" << std::endl;
				std::cerr << "\t ... Found camera with serial number " << m_device_serial_number << " at index " << real_device_index << std::endl;
				break;
			}
		}
		if (real_device_index < 0)
		{
			std::cerr << "ERROR - SoftkineticCamera::Open:" << std::endl;
			std::cerr << "\t ... Could not find device matching serial number to [" << m_device_serial_number << "]." << std::endl;
			return ipa_Utils::RET_FAILED;
		}
	}
	////////////////////////////////////////////////////////////////////////////////
	// Get the desired device
	DepthSense::Device& device = devices[real_device_index];
	std::cerr << "INFO - SoftkineticCamera::Open:" << std::endl;
	std::cerr << "\t ... Configuring camera with index " << real_device_index << " and serial number [" << device.getSerialNumber() << "]" << std::endl;
	// Configure the device
	Callback<void(DepthSense::Device,DepthSense::Device::NodeAddedData)>::func = std::bind(&SoftkineticCamera::OnNodeConnected, this, std::placeholders::_1, std::placeholders::_2);
	callback_nodeadd_t func_nodeadd = static_cast<callback_nodeadd_t>(Callback<void(DepthSense::Device,DepthSense::Device::NodeAddedData)>::callback);
	device.nodeAddedEvent().connect(func_nodeadd);
	Callback<void(DepthSense::Device,DepthSense::Device::NodeRemovedData)>::func = std::bind(&SoftkineticCamera::OnNodeRemoved, this, std::placeholders::_1, std::placeholders::_2);
	callback_noderemove_t func_noderemove = static_cast<callback_noderemove_t>(Callback<void(DepthSense::Device,DepthSense::Device::NodeRemovedData)>::callback);
	device.nodeRemovedEvent().connect(func_noderemove);
	// Configure the first time - it may not initialize properly this time, so we do it twice
	// Get the nodes of the device
	std::vector<DepthSense::Node> device_nodes = device.getNodes();
	// Configure the nodes
	for (size_t idx = 0; idx < device_nodes.size(); idx++)
	{
		DepthSense::Node& node = device_nodes[idx];
		if (node.is<DepthSense::DepthNode>())
		{
			m_depth_node = node.as<DepthSense::DepthNode>();
			ConfigureDepthNode(m_depth_node);
			m_context.registerNode(m_depth_node);
		}
		if (node.is<DepthSense::ColorNode>())
		{
			m_color_node = node.as<DepthSense::ColorNode>();
			ConfigureColorNode(m_color_node);
			m_context.registerNode(m_color_node);
		}
	}
	m_context.startNodes();
	m_open = true;
	Close();
	////////////////////////////////////////////////////////////////////////////////
	// Configure the second time - it should properly initialize this time
	// Get the nodes of the device
	device_nodes = device.getNodes();
	// Configure the nodes
	for (size_t idx = 0; idx < device_nodes.size(); idx++)
	{
		DepthSense::Node& node = device_nodes[idx];
		if (node.is<DepthSense::DepthNode>())
		{
			m_depth_node = node.as<DepthSense::DepthNode>();
			ConfigureDepthNode(m_depth_node);
			m_context.registerNode(m_depth_node);
		}
		if (node.is<DepthSense::ColorNode>())
		{
			m_color_node = node.as<DepthSense::ColorNode>();
			ConfigureColorNode(m_color_node);
			m_context.registerNode(m_color_node);
		}
	}
	////////////////////////////////////////////////////////////////////////////////
	m_context.startNodes();
	std::cerr << "INFO - SoftkineticCamera::Open:" << std::endl;
	std::cerr << "\t ... Starting camera stream." << std::endl;
	
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

	m_context.stopNodes();
	if (m_color_node.isSet())
	{
		Callback<void(DepthSense::ColorNode, DepthSense::ColorNode::NewSampleReceivedData)>::func = std::bind(&SoftkineticCamera::OnNewColorSample, this, std::placeholders::_1, std::placeholders::_2);
		callback_newcolorsample_t func_newcolorsample = static_cast<callback_newcolorsample_t>(Callback<void(DepthSense::ColorNode, DepthSense::ColorNode::NewSampleReceivedData)>::callback);
		m_color_node.newSampleReceivedEvent().disconnect(func_newcolorsample);
		m_context.releaseControl(m_color_node);
		m_context.unregisterNode(m_color_node);
	}
	if (m_depth_node.isSet())
	{
		Callback<void(DepthSense::DepthNode, DepthSense::DepthNode::NewSampleReceivedData)>::func = std::bind(&SoftkineticCamera::OnNewDepthSample, this, std::placeholders::_1, std::placeholders::_2);
		callback_newdepthsample_t func_newdepthsample = static_cast<callback_newdepthsample_t>(Callback<void(DepthSense::DepthNode, DepthSense::DepthNode::NewSampleReceivedData)>::callback);
		m_depth_node.newSampleReceivedEvent().disconnect(func_newdepthsample);
		m_context.releaseControl(m_depth_node);
		m_context.unregisterNode(m_depth_node);
	}

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

void SoftkineticCamera::OnNodeConnected(DepthSense::Device device, DepthSense::Device::NodeAddedData data)
{
	UNUSED(device);
	if (data.node.is<DepthSense::DepthNode>())
	{
		m_depth_node = data.node.as<DepthSense::DepthNode>();
		ConfigureDepthNode(m_depth_node);
		m_context.registerNode(m_depth_node);
	}
	if (data.node.is<DepthSense::ColorNode>())
	{
		m_color_node = data.node.as<DepthSense::ColorNode>();
		ConfigureColorNode(m_color_node);
		m_context.registerNode(m_color_node);
	}
	
	std::cerr << "INFO - SoftkineticCamera::OnNodeConnected:" << std::endl;
	std::cerr << "\t ... Node connected." << std::endl;
}

void SoftkineticCamera::OnNodeRemoved(DepthSense::Device device, DepthSense::Device::NodeRemovedData data)
{
	UNUSED(device);
	if (data.node.is<DepthSense::ColorNode>() && (data.node.as<DepthSense::ColorNode>() == m_color_node))
	{
		Callback<void(DepthSense::ColorNode, DepthSense::ColorNode::NewSampleReceivedData)>::func = std::bind(&SoftkineticCamera::OnNewColorSample, this, std::placeholders::_1, std::placeholders::_2);
		callback_newcolorsample_t func_newcolorsample = static_cast<callback_newcolorsample_t>(Callback<void(DepthSense::ColorNode, DepthSense::ColorNode::NewSampleReceivedData)>::callback);
		m_color_node.newSampleReceivedEvent().disconnect(func_newcolorsample);
		m_color_node.unset();
	}
	if (data.node.is<DepthSense::DepthNode>() && (data.node.as<DepthSense::DepthNode>() == m_depth_node))
	{
		Callback<void(DepthSense::DepthNode, DepthSense::DepthNode::NewSampleReceivedData)>::func = std::bind(&SoftkineticCamera::OnNewDepthSample, this, std::placeholders::_1, std::placeholders::_2);
		callback_newdepthsample_t func_newdepthsample = static_cast<callback_newdepthsample_t>(Callback<void(DepthSense::DepthNode, DepthSense::DepthNode::NewSampleReceivedData)>::callback);
		m_depth_node.newSampleReceivedEvent().disconnect(func_newdepthsample);
		m_depth_node.unset();
	}
	std::cerr << "INFO - SoftkineticCamera::OnNodeRemoved:" << std::endl;
	std::cerr << "\t ... Node removed." << std::endl;
}

void SoftkineticCamera::ConfigureDepthNode(DepthSense::DepthNode& depth_node)
{
	std::cerr << "INFO - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
	std::cerr << "\t ... Configuring new DepthNode ..." << std::endl;
	// Register the event handler
	Callback<void(DepthSense::DepthNode, DepthSense::DepthNode::NewSampleReceivedData)>::func = std::bind(&SoftkineticCamera::OnNewDepthSample, this, std::placeholders::_1, std::placeholders::_2);
	callback_newdepthsample_t func_newdepthsample = static_cast<callback_newdepthsample_t>(Callback<void(DepthSense::DepthNode, DepthSense::DepthNode::NewSampleReceivedData)>::callback);
	depth_node.newSampleReceivedEvent().connect(func_newdepthsample);
	// Try to configure the node
	try
	{
		m_context.requestControl(depth_node, 0);
		depth_node.setConfiguration(m_depth_config.base_config);
		depth_node.setDepthMap3Planes(m_depth_config.enable_xyz_depth);
		depth_node.setDepthMapFloatingPoint3Planes(m_depth_config.enable_floatingpoint_xyz_depth);
		depth_node.setEnableAccelerometer(m_depth_config.enable_accelerometer);
		depth_node.setEnableConfidenceMap(m_depth_config.enable_confidence_map);
		depth_node.setEnableDepthMap(m_depth_config.enable_depth_map);
		depth_node.setEnableDepthMapFloatingPoint(m_depth_config.enable_floatingpoint_depth_map);
		depth_node.setEnablePhaseMap(m_depth_config.enable_phase_map);
		depth_node.setEnableUvMap(m_depth_config.enable_uv_map);
		depth_node.setEnableVertices(m_depth_config.enable_vertices);
		depth_node.setEnableVerticesFloatingPoint(m_depth_config.enable_floatingpoint_vertices);
		if (depth_node.illuminationLevelIsReadOnly())
		{
			std::cerr << "WARN - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
			std::cerr << "\t ... Illumination level cannot be set on this camera." << std::endl;
		}
		else
		{
			depth_node.setIlluminationLevel(m_depth_config.illumination_level);
		}
		std::cerr << "INFO - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... New DepthNode configuration applied." << std::endl;
	}
	catch (DepthSense::ArgumentException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... Argument exception [" << e.what() << "] caught when attempting to configure depth node." << std::endl;
	}
	catch (DepthSense::UnauthorizedAccessException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... Unauthorized access exception [" << e.what() << "] caught when attempting to configure depth node." << std::endl;
	}
	catch (DepthSense::IOException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... IO exception [" << e.what() << "] caught when attempting to configure depth node." << std::endl;
	}
	catch (DepthSense::InvalidOperationException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... Invalid operation exception [" << e.what() << "] caught when attempting to configure depth node." << std::endl;
	}
	catch (DepthSense::ConfigurationException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... Configuration exception [" << e.what() << "] caught when attempting to configure depth node." << std::endl;
	}
	catch (DepthSense::StreamingException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... Streaming exception [" << e.what() << "] caught when attempting to configure depth node." << std::endl;
	}
	catch (DepthSense::TimeoutException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... Timeout exception [" << e.what() << "] caught when attempting to configure depth node." << std::endl;
	}
	catch (DepthSense::Exception& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureDepthNode:" << std::endl;
		std::cerr << "\t ... Unplanned exception [" << e.what() << "] caught when attempting to configure depth node." << std::endl;
	}
}

void SoftkineticCamera::ConfigureColorNode(DepthSense::ColorNode& color_node)
{
	std::cerr << "INFO - SoftkineticCamera::ConfigureColorNode:" << std::endl;
	std::cerr << "\t ... Configuring new ColorNode ..." << std::endl;
	// Register the event handler
	Callback<void(DepthSense::ColorNode, DepthSense::ColorNode::NewSampleReceivedData)>::func = std::bind(&SoftkineticCamera::OnNewColorSample, this, std::placeholders::_1, std::placeholders::_2);
	callback_newcolorsample_t func_newcolorsample = static_cast<callback_newcolorsample_t>(Callback<void(DepthSense::ColorNode, DepthSense::ColorNode::NewSampleReceivedData)>::callback);
	color_node.newSampleReceivedEvent().connect(func_newcolorsample);
	// Try to configure the node
	try
	{
		m_context.requestControl(color_node, 0);
		color_node.setConfiguration(m_color_config.base_config);
		color_node.setBrightness(m_color_config.brightness);
		color_node.setContrast(m_color_config.contrast);
		color_node.setEnableColorMap(m_color_config.enable_color_map);
		color_node.setEnableCompressedData(m_color_config.enable_compressed_data);
		color_node.setGamma(m_color_config.gamma);
		color_node.setHue(m_color_config.hue);
		color_node.setSaturation(m_color_config.saturation);
		color_node.setSharpness(m_color_config.sharpness);
		// Set the exposure
		if (color_node.exposureAutoIsReadOnly() || color_node.exposureAutoPriorityIsReadOnly() || color_node.exposureIsReadOnly())
		{
			std::cerr << "WARN - SoftkineticCamera::ConfigureColorNode:" << std::endl;
			std::cerr << "\t ... Exposure cannot be set on this camera." << std::endl;
		}
		else
		{
			if (m_color_config.enable_priority_auto_exposure_mode)
			{
				color_node.setExposureAuto(DepthSense::EXPOSURE_AUTO_APERTURE_PRIORITY);
				color_node.setExposureAutoPriority(true);
			}
			else
			{
				color_node.setExposureAuto(DepthSense::EXPOSURE_AUTO_MANUAL);
				color_node.setExposureAutoPriority(false);
				color_node.setExposure(m_color_config.exposure);
			}
		}
		// Set the white balance
		if (m_color_config.enable_auto_white_balance)
		{
			color_node.setWhiteBalanceAuto(true);
		}
		else
		{
			color_node.setWhiteBalanceAuto(false);
			color_node.setWhiteBalance(m_color_config.white_balance);
		}
		std::cerr << "INFO - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... New ColorNode configuration applied." << std::endl;
	}
	catch (DepthSense::ArgumentException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... Argument exception [" << e.what() << "] caught when attempting to configure color node." << std::endl;
	}
	catch (DepthSense::UnauthorizedAccessException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... Unauthorized access exception [" << e.what() << "] caught when attempting to configure color node." << std::endl;
	}
	catch (DepthSense::IOException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... IO exception [" << e.what() << "] caught when attempting to configure color node." << std::endl;
	}
	catch (DepthSense::InvalidOperationException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... Invalid operation exception [" << e.what() << "] caught when attempting to configure color node." << std::endl;
	}
	catch (DepthSense::ConfigurationException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... Configuration exception [" << e.what() << "] caught when attempting to configure color node." << std::endl;
	}
	catch (DepthSense::StreamingException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... Streaming exception [" << e.what() << "] caught when attempting to configure color node." << std::endl;
	}
	catch (DepthSense::TimeoutException& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... Timeout exception [" << e.what() << "] caught when attempting to configure color node." << std::endl;
	}
	catch (DepthSense::Exception& e)
	{
		std::cerr << "ERROR - SoftkineticCamera::ConfigureColorNode:" << std::endl;
		std::cerr << "\t ... Unplanned exception [" << e.what() << "] caught when attempting to configure color node." << std::endl;
	}
}


void SoftkineticCamera::OnNewDepthSample(DepthSense::DepthNode node, DepthSense::DepthNode::NewSampleReceivedData data)
{
	UNUSED(node);
	//ros::Time depth_timestamp = ros::Time::now();
	//// Setup the camerainfo messages if they aren't already populated
	//if (g_rgb_camerainfo_set == false)
	//{
	//	SetupCameraInfo(g_rgb_camerainfo, data.stereoCameraParameters.colorIntrinsics);
	//	g_rgb_camerainfo_set = true;
	//	ROS_INFO("Set RGB CameraInfo with parameters provided by the camera");
	//}
	//if (g_depth_camerainfo_set == false)
	//{
	//	SetupCameraInfo(g_depth_camerainfo, data.stereoCameraParameters.depthIntrinsics);
	//	g_depth_camerainfo_set = true;
	//	ROS_INFO("Set Depth CameraInfo with parameters provided by the camera");
	//}

	// First, we generate the raw depth image
	int32_t width = 0;
	int32_t height = 0;
	DepthSense::FrameFormat_toResolution(data.captureConfiguration.frameFormat, &width, &height);

	// Copy and build the depth image
	float* raw_depth_floats = const_cast<float*>(static_cast<const float*>(data.depthMapFloatingPoint));
	std::vector<float> depth_floats(raw_depth_floats, raw_depth_floats + (width * height));
	cv::Mat new_image_depth = cv::Mat(depth_floats).reshape(1, height);
	// Filter the depth image
	cv::Mat new_image_depth_filtered(new_image_depth.rows, new_image_depth.cols, CV_32FC1);
	cv::Mat depth_mean_kernel(3, 3, CV_32FC1);
	depth_mean_kernel.setTo(1.0);
	cv::filter2D(new_image_depth, new_image_depth_filtered, CV_32FC1, depth_mean_kernel);
	m_current_depth_image_mutex.lock();
	m_current_depth_image = new_image_depth_filtered.clone();
	m_current_depth_image_mutex.unlock();

	// retrieve the registered cartesian image (i.e. the 3d measurements are mapped to the right pixel coordinates of the color image)
	// get the (x,y,z) vertices
	DepthSense::FPVertex* raw_vertices = const_cast<DepthSense::FPVertex*>(static_cast<const DepthSense::FPVertex*>(data.verticesFloatingPoint));
	std::vector<DepthSense::FPVertex> vertices(raw_vertices, raw_vertices + (width * height));
	// Get the UV map that maps the color map and vertices together
	DepthSense::UV* raw_uv = const_cast<DepthSense::UV*>(static_cast<const DepthSense::UV*>(data.uvMap));
	std::vector<DepthSense::UV> uv(raw_uv, raw_uv + (width * height));
	// Copy and build the confidences
	int16_t* raw_confidence_shorts = const_cast<int16_t*>(static_cast<const int16_t*>(data.confidenceMap));
	std::vector<int16_t> confidence_shorts(raw_confidence_shorts, raw_confidence_shorts + (width * height));
	// assemble the data
	m_current_cartesian_image_mutex.lock();
	m_current_cartesian_image = cv::Mat::zeros(height, width, CV_32FC3);
	for (size_t idx = 0; idx < vertices.size(); idx++)
	{
		float x = vertices[idx].x;
		float y = -vertices[idx].y;
		float z = vertices[idx].z;
		if (is_vertex_valid(x, y, z))
		{
			float uf = uv[idx].u;
			float vf = uv[idx].v;
			if (is_uv_valid(uf, vf))
			{
				// Filter based on confidence
				int32_t confidence = (int32_t)confidence_shorts[idx];
				if (confidence >= m_confidence_threshold)
				{
					// Make the point
					int v = (int)(vf * height);
					int u = (int)(uf * width);
					m_current_cartesian_image.at<cv::Vec3f>(v,u) = cv::Vec3f(x, y, z);
				}
			}
		}
	}
	m_current_cartesian_image_mutex.unlock();

	//// Second, generate the pointcloud
	//DepthSense::FPVertex* raw_vertices = const_cast<DepthSense::FPVertex*>(static_cast<const DepthSense::FPVertex*>(data.verticesFloatingPoint));
	//std::vector<DepthSense::FPVertex> vertices(raw_vertices, raw_vertices + (width * height));
	//// Make the XYZ pointcloud
	//pcl::PointCloud<pcl::PointXYZI> pcl_pointcloud;
	//for (size_t idx = 0; idx < vertices.size(); idx++)
	//{
	//	float x = vertices[idx].x;
	//	float y = -vertices[idx].y;
	//	float z = vertices[idx].z;
	//	if (is_vertex_valid(x, y, z))
	//	{
	//		// Filter based on confidence
	//		int32_t confidence = (int32_t)confidence_shorts[idx];
	//		if (confidence >= g_confidence_threshold)
	//		{
	//			pcl::PointXYZI new_point;
	//			new_point.x = x;
	//			new_point.y = y;
	//			new_point.z = z;
	//			new_point.intensity = (float)confidence;
	//			pcl_pointcloud.push_back(new_point);
	//		}
	//	}
	//}

	//// Make the XYZRGB pointcloud (if enabled)
	//if (m_color_config.enable_color_map && !m_current_color_image.empty())
	//{
	//	// Get the UV map that maps the color map and vertices together
	//	DepthSense::UV* raw_uv = const_cast<DepthSense::UV*>(static_cast<const DepthSense::UV*>(data.uvMap));
	//	std::vector<DepthSense::UV> uv(raw_uv, raw_uv + (width * height));
	//	// Make the XYZRGB pointcloud
	//	pcl::PointCloud<pcl::PointXYZRGB> pcl_rgb_pointcloud;
	//	// Make the matching UV map
	//	pcl::PointCloud<pcl::PointUV> pcl_uv_pointcloud;
	//	for (size_t idx = 0; idx < vertices.size(); idx++)
	//	{
	//		float x = vertices[idx].x;
	//		float y = -vertices[idx].y;
	//		float z = vertices[idx].z;
	//		float u = uv[idx].u;
	//		float v = uv[idx].v;
	//		if (is_vertex_valid(x, y, z) && is_uv_valid(u, v))
	//		{
	//			// Lookup RGB colors
	//			size_t image_height = (size_t)(v * g_current_color_image.rows);
	//			size_t image_width = (size_t)(u * g_current_color_image.cols);
	//			cv::Vec3b pixel = g_current_color_image.at<cv::Vec3b>(image_height, image_width);
	//			uint8_t blue = pixel[0];
	//			uint8_t green = pixel[1];
	//			uint8_t red = pixel[2];
	//			// Filter based on confidence
	//			int32_t confidence = (int32_t)confidence_shorts[idx];
	//			if (confidence >= g_confidence_threshold)
	//			{
	//				// Make the point
	//				pcl::PointXYZRGB new_xyzrgb_point;
	//				new_xyzrgb_point.x = x;
	//				new_xyzrgb_point.y = y;
	//				new_xyzrgb_point.z = z;
	//				new_xyzrgb_point.r = red;
	//				new_xyzrgb_point.g = green;
	//				new_xyzrgb_point.b = blue;
	//				pcl_rgb_pointcloud.push_back(new_xyzrgb_point);
	//				// Make the UV
	//				pcl::PointUV new_uv_point;
	//				new_uv_point.u = u;
	//				new_uv_point.v = v;
	//				pcl_uv_pointcloud.push_back(new_uv_point);
	//			}
	//		}
	//	}
	//	//// Publish the cloud without filtering
	//	//// Convert the XYZRGB pointcloud to ROS message
	//	//sensor_msgs::PointCloud2 ros_rgb_pointcloud;
	//	//pcl::toROSMsg(pcl_rgb_pointcloud, ros_rgb_pointcloud);
	//	//ros_rgb_pointcloud.header.frame_id = g_depth_optical_frame_name;
	//	//ros_rgb_pointcloud.header.stamp = depth_timestamp;
	//	//g_rgb_pointcloud_pub.publish(ros_rgb_pointcloud);
	//	//// Convert the UV pointcloud to ROS message
	//	//sensor_msgs::PointCloud2 ros_uv_pointcloud;
	//	//pcl::toROSMsg(pcl_uv_pointcloud, ros_uv_pointcloud);
	//	//ros_uv_pointcloud.header.frame_id = g_depth_optical_frame_name;
	//	//ros_uv_pointcloud.header.stamp = depth_timestamp;
	//	//g_uv_pointcloud_pub.publish(ros_uv_pointcloud);
	//	// Make the "registered" depth image
	//	cv::Mat new_image_depth_registered(new_image_depth.rows, new_image_depth.cols, CV_32FC3, cv::Scalar(0.0, 0.0, 0.0));
	//	// Make the "index image" that maps pixels in the image to the corresponding index in the pointcloud
	//	cv::Mat new_image_depth_indices(new_image_depth.rows, new_image_depth.cols, CV_32SC2, cv::Scalar(-1, -1));
	//	int32_t xyz_filtered_vertex_index = 0;
	//	int32_t uv_and_xyz_filtered_vertex_index = 0;
	//	for (size_t idx = 0; idx < vertices.size(); idx++)
	//	{
	//		// Filter based on confidence
	//		int32_t confidence = (int32_t)confidence_shorts[idx];
	//		if (confidence >= g_confidence_threshold)
	//		{
	//			float x = vertices[idx].x;
	//			float y = -vertices[idx].y;
	//			float z = vertices[idx].z;
	//			float u = uv[idx].u;
	//			float v = uv[idx].v;
	//			if (is_vertex_valid(x, y, z))
	//			{
	//				if (is_uv_valid(u, v))
	//				{
	//					// Get the matching position in the image
	//					size_t image_height = (size_t)(v * new_image_depth_registered.rows);
	//					size_t image_width = (size_t)(u * new_image_depth_registered.cols);
	//					// Set that location in the image with the current vertex
	//					new_image_depth_registered.at<cv::Vec3f>(image_height, image_width) = cv::Vec3f(x, y, z);
	//					new_image_depth_indices.at<cv::Vec2i>(image_height, image_width) = cv::Vec2i(xyz_filtered_vertex_index, uv_and_xyz_filtered_vertex_index);
	//					uv_and_xyz_filtered_vertex_index++;
	//				}
	//				xyz_filtered_vertex_index++;
	//			}
	//		}
	//	}
	//	//// Convert the "registered" depth image to ROS and publish it
	//	//std_msgs::Header new_registered_depth_image_header;
	//	//new_registered_depth_image_header.frame_id = g_depth_optical_frame_name;
	//	//new_registered_depth_image_header.stamp = depth_timestamp;
	//	//sensor_msgs::Image new_registed_depth_image;
	//	//cv_bridge::CvImage new_registered_depth_image_converted(new_registered_depth_image_header, sensor_msgs::image_encodings::TYPE_32FC3, new_image_depth_registered);
	//	//new_registered_depth_image_converted.toImageMsg(new_registed_depth_image);
	//	//sensor_msgs::CameraInfo new_registered_depth_image_camerainfo = g_depth_camerainfo;
	//	//new_registered_depth_image_camerainfo.header = new_registered_depth_image_header;
	//	//// Publish the image
	//	//g_registered_depth_pub.publish(new_registed_depth_image, new_registered_depth_image_camerainfo);
	//	//// Convert the "index image" to ROS and publish it
	//	//std_msgs::Header new_indices_depth_image_header;
	//	//new_indices_depth_image_header.frame_id = g_depth_optical_frame_name;
	//	//new_indices_depth_image_header.stamp = depth_timestamp;
	//	//sensor_msgs::Image new_indices_depth_image;
	//	//cv_bridge::CvImage new_indices_depth_image_converted(new_indices_depth_image_header, sensor_msgs::image_encodings::TYPE_32SC2, new_image_depth_indices);
	//	//new_indices_depth_image_converted.toImageMsg(new_indices_depth_image);
	//	//sensor_msgs::CameraInfo new_indices_depth_image_camerainfo = g_depth_camerainfo;
	//	//new_indices_depth_image_camerainfo.header = new_indices_depth_image_header;
	//	//// Publish the image
	//	//g_index_pub.publish(new_indices_depth_image, new_indices_depth_image_camerainfo);
	//}
	//else if (g_color_config.enable_color_map && g_current_color_image.empty())
	//{
	//	ROS_WARN("XYZRGB pointclouds enabled, but no color image received yet. Not publishing XYZRGB pointcloud");
	//}
}

inline bool SoftkineticCamera::is_vertex_valid(float x, float y, float z)
{
    if (x != -2.0 || y != -2.0 || z != -2.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

inline bool SoftkineticCamera::is_uv_valid(float u, float v)
{
    if (u != -FLT_MAX || v != -FLT_MAX)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void SoftkineticCamera::OnNewColorSample(DepthSense::ColorNode node, DepthSense::ColorNode::NewSampleReceivedData data)
{
	UNUSED(node);
	// Make new OpenCV container
	int32_t width = 0;
	int32_t height = 0;
	DepthSense::FrameFormat_toResolution(data.captureConfiguration.frameFormat, &width, &height);
	cv::Mat new_image_bgr8(height, width, CV_8UC3);
	// If the image is in YUY2 mode, we need to convert it to BGR8 first
	if (data.captureConfiguration.compression == DepthSense::COMPRESSION_TYPE_YUY2)
	{
		// Make the intermediate container for the YUY2 image
		cv::Mat new_image_yuy2(height, width, CV_8UC2);
		// Copy the data in
		new_image_yuy2.data = const_cast<uint8_t*>(static_cast<const uint8_t*>(data.colorMap));
		// Convert to BGR
		cv::cvtColor(new_image_yuy2, new_image_bgr8, CV_YUV2BGR_YUY2);
	}
	// If the image is in MJPEG mode, it's already in BGR8
	else
	{
		// Copy the data in
		new_image_bgr8.data = const_cast<uint8_t*>(static_cast<const uint8_t*>(data.colorMap));
	}
	// Save the current color image
	m_current_color_image_mutex.lock();
	m_current_color_image = new_image_bgr8.clone();
	m_current_color_image_mutex.unlock();

	//// Convert the OpenCV image to ROS
	//std_msgs::Header new_image_header;
	//new_image_header.frame_id = g_rgb_optical_frame_name;
	//new_image_header.stamp = ros::Time::now();
	//sensor_msgs::Image new_image;
	//cv_bridge::CvImage new_image_converted(new_image_header, sensor_msgs::image_encodings::BGR8, new_image_bgr8);
	//new_image_converted.toImageMsg(new_image);
	//if (g_rgb_camerainfo_set == true)
	//{
	//	sensor_msgs::CameraInfo new_image_camerainfo = g_rgb_camerainfo;
	//	new_image_camerainfo.header = new_image_header;
	//	// Publish the image
	//	g_rgb_pub.publish(new_image, new_image_camerainfo);
	//}
	//else
	//{
	//	ROS_WARN("No RGB CameraInfo set, not publishing RGB image");
	//}
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
			cameraProperty->propertyType = TYPE_CAMERA_RESOLUTION;
			if (isOpen())
			{
				int32_t width = 0;
				int32_t height = 0;
				DepthSense::FrameFormat_toResolution(m_color_config.base_config.frameFormat, &width, &height);
				cameraProperty->cameraResolution.xResolution = width;
				cameraProperty->cameraResolution.yResolution = height;
			}
			else
			{
				std::cout << "WARNING - SoftkineticCamera::GetProperty:" << std::endl;
				std::cout << "\t ... Camera not open" << std::endl;
				std::cout << "\t ... Returning default width and height of '" << 1280 << "' x '" << 720 << "'" << std::endl;
				cameraProperty->cameraResolution.xResolution = 1280;
				cameraProperty->cameraResolution.yResolution = 720;
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
	
	// rescale everything to the color image resolution
	int32_t width = 0;
	int32_t height = 0;
	DepthSense::FrameFormat_toResolution(m_color_config.base_config.frameFormat, &width, &height);

	if(rangeImage)
	{
		// Depth image is upsampled according to the size of the color image
		rangeImage->create(height, width, CV_32FC1);
		rangeImageData = rangeImage->ptr<char>(0);
		widthStepRange = rangeImage->step;
	}
	
	if(colorImage)
	{
		colorImage->create(height, width, CV_8UC3);
		colorImageData = colorImage->ptr<char>(0);
		widthStepColor = colorImage->step;
	}	

	if(cartesianImage)
	{
		// Depth image is upsampled according to the size of the color image
		cartesianImage->create(height, width, CV_32FC3);
		cartesianImageData = cartesianImage->ptr<char>(0);
		widthStepCartesian = cartesianImage->step;
	}

	if (!rangeImage && !colorImage && !cartesianImage)
		return RET_OK;

	return AcquireImages(widthStepRange, widthStepColor, widthStepCartesian, rangeImageData, colorImageData,  cartesianImageData, getLatestFrame, undistort, grayImageType);
}

unsigned long SoftkineticCamera::AcquireImages(int widthStepRange, int widthStepGray, int widthStepCartesian, char* rangeImageData, char* colorImageData, char* cartesianImageData,
										bool getLatestFrame, bool undistort, ipa_CameraSensors::t_ToFGrayImageType grayImageType)
{
	// rescale everything to the color image resolution
	int32_t width = 0;
	int32_t height = 0;
	DepthSense::FrameFormat_toResolution(m_color_config.base_config.frameFormat, &width, &height);

	cv::Mat cartesian_image;
	m_current_cartesian_image_mutex.lock();
	if (m_current_cartesian_image.rows != height || m_current_cartesian_image.cols != width)
		cv::resize(m_current_cartesian_image, cartesian_image, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
	else
		cartesian_image = m_current_cartesian_image.clone();
	m_current_cartesian_image_mutex.unlock();

	cv::Mat range_image;
	m_current_depth_image_mutex.lock();
	if (m_current_depth_image.rows != height || m_current_depth_image.cols != width)
		cv::resize(m_current_depth_image, range_image, cv::Size(width, height), 0, 0, cv::INTER_LINEAR);
	else
		range_image = m_current_depth_image.clone();
	m_current_depth_image_mutex.unlock();

	cv::imshow("m_current_color_image", m_current_color_image);
	cv::waitKey();

	if (colorImageData)
	{
		m_current_color_image_mutex.lock();
		unsigned char* p_colorImageData = (unsigned char*)colorImageData;
		for (int v=0; v<height; ++v)
		{
			unsigned char* p_srcImageData = (unsigned char*)m_current_color_image.ptr(v);
			for (int u=0; u<width; ++u)
			{
				*p_colorImageData = *p_srcImageData;
				++p_colorImageData; ++p_srcImageData;
				*p_colorImageData = *p_srcImageData;
				++p_colorImageData; ++p_srcImageData;
				*p_colorImageData = *p_srcImageData;
				++p_colorImageData; ++p_srcImageData;
			}
		}
		m_current_color_image_mutex.unlock();
	}
	std::cout << "1: " << cartesian_image.rows << "x" << cartesian_image.cols << "\t" << cartesian_image.at<cv::Vec3f>(718,974).val[0] <<  std::endl;
	cv::imshow("cartesian_image", cartesian_image);
	cv::waitKey();
	if (cartesianImageData)
	{
		float* p_cartesianImageData = (float*)cartesianImageData;
		for (int v=0; v<height; ++v)
		{
			std::cout << v << std::endl;
			float* p_srcImageData = (float*)cartesian_image.ptr(v);
			for (int u=0; u<width; ++u)
			{
				if (v>=718)	// 974
					std::cout << "u=" << u << std::endl;
				*p_cartesianImageData = 0.f;//cartesian_image.at<cv::Vec3f>(v,u).val[0];	//*p_srcImageData;
				++p_cartesianImageData; //++p_srcImageData;
				*p_cartesianImageData = 0.f;//cartesian_image.at<cv::Vec3f>(v,u).val[1];	//*p_srcImageData;
				++p_cartesianImageData; //++p_srcImageData;
				*p_cartesianImageData = 0.f;//cartesian_image.at<cv::Vec3f>(v,u).val[2];	//*p_srcImageData;
				++p_cartesianImageData; //++p_srcImageData;
			}
		}
	}
	std::cout << "2" << std::endl;
	cv::imshow("range_image", range_image);
	cv::waitKey();
	if (rangeImageData)
	{
		float* p_rangeImageData = (float*)rangeImageData;
		for (int v=0; v<height; ++v)
		{
			float* p_srcImageData = (float*)range_image.ptr(v);
			for (int u=0; u<width; ++u)
			{
				*p_rangeImageData = *p_srcImageData;
				++p_rangeImageData; ++p_srcImageData;
			}
		}
	}

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
					if (m_device_serial_number == "0")
					{
						m_device_serial_number = "";
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
				int32_t color_width = 0;
				int32_t color_height = 0;
				DepthSense::FrameFormat_toResolution(m_color_config.base_config.frameFormat, &color_width, &color_height);
				m_current_color_image_mutex.lock();
				m_current_color_image = cv::Mat::zeros(color_height, color_width, CV_8UC3);
				m_current_color_image_mutex.unlock();

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
				int32_t depth_width = 0;
				int32_t depth_height = 0;
				DepthSense::FrameFormat_toResolution(m_depth_config.base_config.frameFormat, &depth_width, &depth_height);
				m_current_depth_image_mutex.lock();
				m_current_depth_image = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);
				m_current_depth_image_mutex.unlock();
				m_current_cartesian_image_mutex.lock();
				m_current_cartesian_image = cv::Mat::zeros(depth_height, depth_width, CV_8UC3);
				m_current_cartesian_image_mutex.unlock();
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