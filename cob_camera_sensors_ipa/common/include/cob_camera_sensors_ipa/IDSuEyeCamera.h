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

/// @file IDSuEye.h
/// Interface for IDS uEye color cameras.
/// @author Richard Bormann
/// @date November 2015.

#ifndef __IPA_IDSUEYECAMERA_H__
#define __IPA_IDSUEYECAMERA_H__

#ifdef __BUILD_WITH_IDS_UEYE__

#ifdef __LINUX__
	#include "cob_camera_sensors/AbstractColorCamera.h"
#else
	#include "cob_driver/cob_camera_sensors/common/include/cob_camera_sensors/AbstractColorCamera.h"
#endif

#include <opencv2/core/core.hpp>
#include <set>

#include "uEye.h"


namespace ipa_CameraSensors {

class __DLL_LIBCAMERASENSORS__ IDSuEyeCamera : public AbstractColorCamera 
{
	public:

		/// Constructor
		IDSuEyeCamera(const std::string xmlTagName = "IDSuEyeCamera_");
		
		/// Destructor
		~IDSuEyeCamera();

		/// Initializes the color camera.
		/// Camera specific constants may be set within the configuration file <I>cameraSensorsIni.xml</I>.
		/// The function has to set the member variable <code>m_initialized</code>.
		/// @param directory Path to the configuration file directory.
		/// @param cameraIndex It is possible to have several cameras of the same type on the system.
		///	       One may us the camera index to apply different configuration files to each of them
		/// @return Return code.
		unsigned long Init(std::string directory, int cameraIndex = 0);
	
		/// Returns true, when <code>Init()</code> has been called on the camera.
		/// @return Camera initialized or not.
		bool isInitialized() {return m_initialized;}

		/// Returns true, when <code>Open()</code> has been called on the camera.
		/// @return Camera opened or not.
		bool isOpen() {return m_open;}

		/// Opens the camera device.
		/// All camera specific parameters for opening the camera should have been set within the <code>Init</code>
		/// function.
		/// @return Return code.
		unsigned long Open();

		/// Close camera device.
		/// @return Return code.
		unsigned long Close(); //Save intrinsic params back to File
		
		/// Retrieves image data from the color camera.
		/// @param colorImageData An array to be filled with image data
		/// @param getLatestFrame True, when the latest picture has to be returned. Otherwise, the next picture
		///						  following the last call to <code>getLatestFrame</code> is returned.
		/// @return Return code
		unsigned long GetColorImage(char* colorImageData, bool getLatestFrame=true);

		/// Retrieves an image from the camera.
		/// <code>cv::Mat</code> object is initialized on demand.
		/// @param colorImage The image that has been acquired by the camera.
		/// @param getLatestFrame If true, the camera acquires a new frame and returns it.
		///						  Otherwise, the next frame following the last returned frame
		///						  is returned from the internal camera buffer.
		/// @throw IPA_Exception Throws an exception, if camera access failed
		unsigned long GetColorImage(cv::Mat* colorImage, bool getLatestFrame=true);

		/// Returns the camera type.
		/// @return The camera type
		t_cameraType GetCameraType() { return m_CameraType; }
	
		/// Function to set properties of the camera sensor.
		/// @param propertyID The ID of the property.
		/// @param cameraProperty The value of the property.
		/// @return Return code.
		unsigned long SetProperty(t_cameraProperty* cameraProperty) {return RET_FAILED;};

		/// Function to set property defaults of the camera sensor.
		/// @return Return code.
		unsigned long SetPropertyDefaults() {return RET_FAILED;};

		/// Function to get properties of the camera sensor.
		/// @param propertyID The ID of the property.
		/// @param cameraProperty The value of the property.
		/// @return Return code.
		unsigned long GetProperty(t_cameraProperty* cameraProperty);

		/// Displays camera information on standard output.
		/// Information includes available parameters, color and camera formats.
		/// @return Return code.
		unsigned long PrintCameraInformation() {return RET_FAILED;};

		/// Saves all parameters on hard disk.
		/// @param filename The filename of the storage.
		/// @return Return code.
		unsigned long SaveParameters(const char* filename) {return RET_FAILED;};

		/// Unit Test for the camera interface.
		/// Tests each of the single interface functions and displays the output on
		/// standard out.
		/// @param filename Path to the camera initialization xml file.
		/// @return Return code.
		unsigned long TestCamera(const char* filename);

		/// Returns the number of images in the directory
		/// @return The number of images in the directory
		int GetNumberOfImages() {return std::numeric_limits<int>::max();};

		/// Function specific to virtual camera.
		/// Resets the image directory read from the configuration file.
		/// @param path The camera path
		/// @return Return code
		unsigned long SetPathToImages(std::string path) {return RET_OK;};

	protected:
		
		bool m_initialized; ///< True, when the camera has sucessfully been initialized.
		bool m_open;		///< True, when the camera has sucessfully been opend.

		HIDS m_cameraDevice;		///< handle to camera
		INT m_pcImageMemoryId;		///< grabber memory - buffer ID
		char* m_pcImageMemory;		///< grabber memory - pointer to buffer

		int m_width;		///< image width
		int m_height;		///< image height

		t_cameraType m_CameraType;	///< Camera Type

		t_ColorCameraParameters m_ColorCameraParameters; ///< Storage for xml configuration file data
		
		unsigned int m_BufferSize; ///< Number of images, the camera buffers internally

		std::string m_xmlTagName;	///< xml tag of this camera in cameraSensors.ini file
		
	private:

		/// Loads all camera specific parameters from the xml configuration file and saves them in t_ColorCameraParameters.
		/// This function is internally called by Init to load the parameters from the xml configuration file.
		/// @param filename The path to the configuration file.
		/// @return Return code.
		unsigned long LoadParameters(const char* filename, int cameraIndex);

		/// Sets the loaded parameters.
		/// @return Return code.
		unsigned long SetParameters() {return RET_OK;};

};

/// Creates, intializes and returns a smart pointer object for the camera.
/// @return Smart pointer, refering to the generated object
__DLL_LIBCAMERASENSORS__ AbstractColorCameraPtr CreateColorCamera_IDSuEyeCamera();


} // end namespace

#endif // __BUILD_WITH_IDS_UEYE__

#endif // __IPA_IDSUEYECAMERA_H__
