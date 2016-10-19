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

/// @file SoftkineticCamera.h
/// Interface for Softkinetic cameras.
/// @author Richard Bormann
/// @date October 2016.

#ifndef __IPA_SOFTKINETICCAMERA_H__
#define __IPA_SOFTKINETICCAMERA_H__

#ifdef __BUILD_WITH_SOFTKINETIC__

#ifdef __LINUX__
	#include <cob_camera_sensors/AbstractRangeImagingSensor.h>
#else
	#include <cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractRangeImagingSensor.h>
#endif

#include <opencv2/core/core.hpp>
#include <set>

#include "DepthSense.hxx"

// trick for using callbacks with non-static member functions of objects
// see http://stackoverflow.com/questions/1000663/using-a-c-class-member-function-as-a-c-callback-function
// specified here for 2 arguments to avoid using C++ 11 standard
	template <typename T>
	struct Callback;

	template <typename Ret, typename Param1, typename Param2>
	struct Callback<Ret(Param1, Param2)>
	{
	   template <typename Arg1, typename Arg2> 
	   static Ret callback(Arg1 arg1, Arg2 arg2)
	   {                    
		  func(arg1, arg2);  
	   }
	   static std::function<Ret(Param1, Param2)> func; 
	};

	template <typename Ret, typename Param1, typename Param2>
	std::function<Ret(Param1, Param2)> Callback<Ret(Param1, Param2)>::func;

	typedef void (*callback_deviceadd_t)(DepthSense::Context,DepthSense::Context::DeviceAddedData);
	typedef void (*callback_deviceremove_t)(DepthSense::Context,DepthSense::Context::DeviceRemovedData);
// end of trick for binding non-static member functions to callbacks

namespace ipa_CameraSensors {

/// @ingroup RangeCameraDriver
/// Platform independent interface to SoftKinetic camera.
class __DLL_LIBCAMERASENSORS__ SoftkineticCamera : public AbstractRangeImagingSensor
{
public:

	typedef struct
	{
		DepthSense::ColorNode::Configuration base_config;
		DepthSense::ExposureAuto auto_exposure_mode;
		int32_t brightness;
		int32_t contrast;
		int32_t saturation;
		int32_t hue;
		int32_t gamma;
		int32_t white_balance;
		int32_t sharpness;
		int32_t exposure;
		bool enable_priority_auto_exposure_mode;
		bool enable_auto_white_balance;
		bool enable_color_map;
		bool enable_compressed_data;
	} CompleteColorConfig;

	typedef struct
	{
		DepthSense::DepthNode::Configuration base_config;
		int32_t illumination_level;
		bool enable_xyz_depth;
		bool enable_floatingpoint_xyz_depth;
		bool enable_accelerometer;
		bool enable_confidence_map;
		bool enable_depth_map;
		bool enable_floatingpoint_depth_map;
		bool enable_phase_map;
		bool enable_uv_map;
		bool enable_vertices;
		bool enable_floatingpoint_vertices;
	} CompleteDepthConfig;

	SoftkineticCamera();
	~SoftkineticCamera();

	//*******************************************************************************
	// AbstractRangeImagingSensor interface implementation
	//*******************************************************************************

	unsigned long Init(std::string directory, int cameraIndex = 0);

	unsigned long Open();
	unsigned long Close();

	unsigned long SetProperty(t_cameraProperty* cameraProperty);
	unsigned long SetPropertyDefaults();
	unsigned long GetProperty(t_cameraProperty* cameraProperty);

	unsigned long AcquireImages(int widthStepRange, int widthStepColor, int widthStepCartesian, char* rangeImage=NULL, char* colorImage=NULL,
		char* cartesianImage=NULL, bool getLatestFrame=true, bool undistort=true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);
	unsigned long AcquireImages(cv::Mat* rangeImage = 0, cv::Mat* colorImage = 0,
		cv::Mat* cartesianImage = 0, bool getLatestFrame = true, bool undistort = true,
		ipa_CameraSensors::t_ToFGrayImageType grayImageType = ipa_CameraSensors::INTENSITY);

	/// Returns the camera type.
	/// @return The camera type
	t_cameraType GetCameraType() { return m_CameraType; }

	unsigned long SaveParameters(const char* filename);

	bool isInitialized() {return m_initialized;}
	bool isOpen() {return m_open;}

private:
	
	t_cameraType m_CameraType;			///< Camera Type

	////*******************************************************************************
	//// Camera specific members
	////*******************************************************************************
	
	int32_t m_device_index;
	std::string m_device_serial_number;

	// DepthSense SDK global variables
	DepthSense::Context m_context;
	DepthSense::ColorNode m_color_node;
	DepthSense::DepthNode m_depth_node;
	CompleteColorConfig m_color_config;
	CompleteDepthConfig m_depth_config;
	
	// Config for the confidence filter
	int32_t m_confidence_threshold;

	// Save the current color map for coloring pointclouds
	cv::Mat current_color_image;
	


	//int m_width;		///< image width
	//int m_height;		///< image height

	std::string m_parameter_files_directory;	///< folder of parameter files
	
	//std::string m_JSONCalibration;		///< JSON calibration string for camera

	void OnDeviceConnected(DepthSense::Context context, DepthSense::Context::DeviceAddedData data);

	void OnDeviceRemoved(DepthSense::Context context, DepthSense::Context::DeviceRemovedData data);

	unsigned long LoadParameters(const char* filename, int cameraIndex);
};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractRangeImagingSensorPtr CreateRangeImagingSensor_SoftkineticCamera();

} // End namespace ipa_CameraSensors

#endif // __BUILD_WITH_SOFTKINETIC__

#endif // __IPA_SOFTKINETICCAMERA_H__
