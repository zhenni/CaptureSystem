//=============================================================================
// Copyright (c) 2001-2018 FLIR Systems, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

/**
*  @example AcquisitionMultipleThread.cpp
*
*  @brief AcquisitionMultipleThread.cpp shows how to capture images from multiple
*  cameras simultaneously using threads. It relies on information provided in the
*  Enumeration, Acquisition, and NodeMapInfo examples.
*
*  This example is similar to the Acquisition example, except that threads
*  are used to allow for simultaneous acquisitions.
*/

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <fstream>
#include <sstream> 
#include "SpinVideo.h"

#ifndef _WIN32
#include <pthread.h>
#endif

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace Spinnaker::Video;
using namespace std;

// Use the following enum and global constant to select the type of video
// file to be created and saved.
enum videoType
{
	UNCOMPRESSED,
	MJPG,
	H264
};

// Use the following enum and global constant to select whether chunk data is 
// displayed from the image or the nodemap.
enum chunkDataType
{
	IMAGE,
	NODEMAP
};


enum bufferType
{
	NewestFirst,
	NewestFirstOverwrite,
	NewestOnly,
	OldestFirst,
	OldestFirstOverwrite,
};


// ===================================================================================
// ==================================== SELECT =======================================
// ===================================================================================
const chunkDataType chosenChunkData = IMAGE;
const PixelFormatEnums savePixelFormat = PixelFormat_BGR8; // PixelFormat_Mono8;
const unsigned int numBuffers = 10; // Total number of buffers
const bufferType chosenBufferType = OldestFirstOverwrite;

const string serialNumberPrimary = "18565847"; // "18566303";
const gcstring grabPixelFormatName = "BayerBG8"; // BayerBG8

const int selectFrameRate = 20;
const videoType chosenVideoType = MJPG; // UNCOMPRESSED;
const unsigned int k_numImages = 10000;
const unsigned int k_numPrintInfo = 20;

// const unsigned int k_savePerNumImages = 100;
// const unsigned int k_threadPerCameraForSaving = 3;
const unsigned int imageHeight = 1028; //???
const unsigned int imageWidth = 1280;

const int k_numCameras = 6;
const string serialNumbers[k_numCameras] = {
	"18565847",
	"18565848",
	"18565849",
	"18565850",
	"18565851",
	"18566303"
};

// const string outputFolder = "F:\\temp\\test_sync\\";
const string outputFolders[k_numCameras] = {
	"D:\\temp\\test_sync\\",
	"G:\\temp\\test_sync\\",
	"H:\\temp\\test_sync\\",
	"I:\\temp\\test_sync\\",
	"J:\\temp\\test_sync\\",
	"K:\\temp\\test_sync\\"
};

const string subfolderName = "022219_0005_test10000frames";
// ===================================================================================


// This helper function allows the example to sleep in both Windows and Linux 
// systems. Note that Windows sleep takes milliseconds as a parameter while
// Linux systems take microseconds as a parameter. 
void SleepyWrapper(int milliseconds)
{
#if defined WIN32 || defined _WIN32 || defined WIN64 || defined _WIN64 
	Sleep(milliseconds);
#else
	usleep(1000 * milliseconds);
#endif
}


// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap & nodeMap, std::string camSerial)
{
	int result = 0;

	cout << "[" << camSerial << "] Printing device information ..." << endl << endl;

	FeatureList_t features;
	CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
	if (IsAvailable(category) && IsReadable(category))
	{
		category->GetFeatures(features);

		FeatureList_t::const_iterator it;
		for (it = features.begin(); it != features.end(); ++it)
		{
			CNodePtr pfeatureNode = *it;
			CValuePtr pValue = (CValuePtr)pfeatureNode;
			cout << "[" << camSerial << "] " << pfeatureNode->GetName() << " : " << (IsReadable(pValue) ? pValue->ToString() : "Node not readable") << endl;
		}
	}
	else
	{
		cout << "[" << camSerial << "] " << "Device control information not available." << endl;
	}

	cout << endl;

	return result;
}

int ConfigureBuffer(INodeMap & sNodeMap)
{
	// Retrieve Stream Parameters device nodemap 
	// Spinnaker::GenApi::INodeMap & sNodeMap = pCam->GetTLStreamNodeMap();

	try
	{
		// Retrieve Buffer Handling Mode Information
		CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
		if (!IsAvailable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
		{
			cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << endl << endl;
			return -1;
		}
		CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
		if (!IsAvailable(ptrHandlingModeEntry) || !IsReadable(ptrHandlingModeEntry))
		{
			cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << endl << endl;
			return -1;
		}

		// Set stream buffer Count Mode to manual
		CEnumerationPtr ptrStreamBufferCountMode = sNodeMap.GetNode("StreamBufferCountMode");
		if (!IsAvailable(ptrStreamBufferCountMode) || !IsWritable(ptrStreamBufferCountMode))
		{
			cout << "Unable to set Buffer Count Mode (node retrieval). Aborting..." << endl << endl;
			return -1;
		}

		CEnumEntryPtr ptrStreamBufferCountModeManual = ptrStreamBufferCountMode->GetEntryByName("Manual"); //  Original: Auto
		if (!IsAvailable(ptrStreamBufferCountModeManual) || !IsReadable(ptrStreamBufferCountModeManual))
		{
			cout << "Unable to set Buffer Count Mode entry (Entry retrieval). Aborting..." << endl << endl;
			return -1;
		}

		ptrStreamBufferCountMode->SetIntValue(ptrStreamBufferCountModeManual->GetValue());

		cout << "Stream Buffer Count Mode set to manual..." << endl;

		// Retrieve and modify Stream Buffer Count
		CIntegerPtr ptrBufferCount = sNodeMap.GetNode("StreamBufferCountManual");
		if (!IsAvailable(ptrBufferCount) || !IsWritable(ptrBufferCount))
		{
			cout << "Unable to set Buffer Count (Integer node retrieval). Aborting..." << endl << endl;
			return -1;
		}

		// Display Buffer Info
		cout << endl << "Default Buffer Handling Mode: " << ptrHandlingModeEntry->GetDisplayName() << endl;
		cout << "Default Buffer Count: " << ptrBufferCount->GetValue() << endl;
		cout << "Maximum Buffer Count: " << ptrBufferCount->GetMax() << endl;

		ptrBufferCount->SetValue(numBuffers);

		cout << "Buffer count now set to: " << ptrBufferCount->GetValue() << endl;

		switch (chosenBufferType)
		{
		case NewestFirst:
			ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestFirst");
			ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());
			cout << endl << endl << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry->GetDisplayName() << endl;
			break;

		case NewestFirstOverwrite:
			ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestFirstOverwrite");
			ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());
			cout << endl << endl << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry->GetDisplayName() << endl;
			break;
		
		case NewestOnly:
			ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly"); // Default setting
			ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());
			cout << endl << endl << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry->GetDisplayName() << endl;
			break;

		case OldestFirst:
			ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("OldestFirst");
			ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());
			cout << endl << endl << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry->GetDisplayName() << endl;
			break;

		case OldestFirstOverwrite:
			ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("OldestFirstOverwrite");
			ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());
			cout << endl << endl << "Buffer Handling Mode has been set to " << ptrHandlingModeEntry->GetDisplayName() << endl;
			break;

		default:
			break;
		}

	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		return -1;
	}

	return 0;
}


// This function disables each type of chunk data before disabling chunk data mode. 
int DisableChunkData(INodeMap & nodeMap)
{
	int result = 0;
	try
	{
		NodeList_t entries;

		// Retrieve the selector node
		CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");

		if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector))
		{
			cout << "Unable to retrieve chunk selector. Aborting..." << endl << endl;
			return -1;
		}

		// Retrieve entries
		ptrChunkSelector->GetEntries(entries);

		cout << "Disabling entries..." << endl;

		for (int i = 0; i < entries.size(); i++)
		{
			// Select entry to be disabled
			CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);

			// Go to next node if problem occurs
			if (!IsAvailable(ptrChunkSelectorEntry) || !IsReadable(ptrChunkSelectorEntry))
			{
				continue;
			}

			ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

			cout << "\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ";

			// Retrieve corresponding boolean
			CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");

			// Disable the boolean, thus disabling the corresponding chunk data
			if (!IsAvailable(ptrChunkEnable))
			{
				cout << "not available" << endl;
				result = -1;
			}
			else if (!ptrChunkEnable->GetValue())
			{
				cout << "disabled" << endl;
			}
			else if (IsWritable(ptrChunkEnable))
			{
				ptrChunkEnable->SetValue(false);
				cout << "disabled" << endl;
			}
			else
			{
				cout << "not writable" << endl;
			}
		}
		cout << endl;

		//Deactivate ChunkMode
		CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive");

		if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive))
		{
			cout << "Unable to deactivate chunk mode. Aborting..." << endl << endl;
			return -1;
		}

		ptrChunkModeActive->SetValue(false);

		cout << "Chunk mode deactivated..." << endl;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}


// This function displays a select amount of chunk data from the image. Unlike
// accessing chunk data via the nodemap, there is no way to loop through all 
// available data.
int DisplayChunkData(ImagePtr pImage, ofstream& logFile, int frame_id)
{
	int result = 0;

	logFile << "Frame ID " << frame_id << "\n";
	try
	{
		//
		// Retrieve chunk data from image
		//
		// *** NOTES ***
		// When retrieving chunk data from an image, the data is stored in a
		// a ChunkData object and accessed with getter functions.
		//
		ChunkData chunkData = pImage->GetChunkData();

		//
		// Retrieve exposure time; exposure time recorded in microseconds
		//
		// *** NOTES ***
		// Floating point numbers are returned as a float64_t. This can safely
		// and easily be statically cast to a double.
		//
		double exposureTime = static_cast<double>(chunkData.GetExposureTime());
		logFile << "\tExposure time: " << exposureTime << "\n";

		//
		// Retrieve frame ID
		//
		// *** NOTES ***
		// Integers are returned as an int64_t. As this is the typical integer
		// data type used in the Spinnaker SDK, there is no need to cast it.
		//
		int64_t frameID = chunkData.GetFrameID();
		logFile << "\tFrame ID: " << frameID << "\n";

		// Retrieve gain; gain recorded in decibels
		double gain = chunkData.GetGain();
		logFile << "\tGain: " << gain << "\n";

		// Retrieve height; height recorded in pixels
		int64_t height = chunkData.GetHeight();
		logFile << "\tHeight: " << height << "\n";

		// Retrieve width; width recorded in pixels
		int64_t width = chunkData.GetWidth();
		logFile << "\tWidth: " << width << "\n";

		// Retrieve offset X; offset X recorded in pixels
		int64_t offsetX = chunkData.GetOffsetX();
		logFile << "\tOffset X: " << offsetX << "\n";

		// Retrieve offset Y; offset Y recorded in pixels
		int64_t offsetY = chunkData.GetOffsetY();
		logFile << "\tOffset Y: " << offsetY << "\n";

		// Retrieve sequencer set active
		int64_t sequencerSetActive = chunkData.GetSequencerSetActive();
		logFile << "\tSequencer set active: " << sequencerSetActive << "\n";

		// Retrieve timestamp
		uint64_t timestamp = chunkData.GetTimestamp();
		logFile << "\tTimestamp: " << timestamp / long int(1e9) << "." << timestamp % long int(1e9) << "\n";

		logFile << endl;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
int DisableHeartbeat(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice)
{
	cout << "Checking device type to see if we need to disable the camera's heartbeat..." << endl << endl;
	//
	// Write to boolean node controlling the camera's heartbeat
	// 
	// *** NOTES ***
	// This applies only to GEV cameras and only applies when in DEBUG mode.
	// GEV cameras have a heartbeat built in, but when debugging applications the
	// camera may time out due to its heartbeat. Disabling the heartbeat prevents 
	// this timeout from occurring, enabling us to continue with any necessary debugging.
	// This procedure does not affect other types of cameras and will prematurely exit
	// if it determines the device in question is not a GEV camera. 
	//
	// *** LATER ***
	// Since we only disable the heartbeat on GEV cameras during debug mode, it is better
	// to power cycle the camera after debugging. A power cycle will reset the camera 
	// to its default settings. 
	// 

	CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
	if (!IsAvailable(ptrDeviceType) && !IsReadable(ptrDeviceType))
	{
		cout << "Error with reading the device's type. Aborting..." << endl << endl;
		return -1;
	}
	else
	{
		if (ptrDeviceType->GetIntValue() == DeviceType_GEV)
		{
			cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << endl << endl;
			CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
			if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
			{
				cout << "Unable to disable heartbeat on camera. Continuing with execution as this may be non-fatal..." << endl << endl;
			}
			else
			{
				ptrDeviceHeartbeat->SetValue(true);
				cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << endl;
				cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << endl << endl;
			}
		}
		else
		{
			cout << "Camera does not use GigE interface. Resuming normal execution..." << endl << endl;
		}
	}
	return 0;
}
#endif


// This function configures the camera to add chunk data to each image. It does 
// this by enabling each type of chunk data before enabling chunk data mode. 
// When chunk data is turned on, the data is made available in both the nodemap 
// and each image.
int ConfigureChunkData(INodeMap & nodeMap)
{
	int result = 0;

	cout << endl << endl << "*** CONFIGURING CHUNK DATA ***" << endl << endl;

	try
	{
		//
		// Activate chunk mode
		//
		// *** NOTES ***
		// Once enabled, chunk data will be available at the end of the payload
		// of every image captured until it is disabled. Chunk data can also be 
		// retrieved from the nodemap.
		//
		CBooleanPtr ptrChunkModeActive = nodeMap.GetNode("ChunkModeActive");

		if (!IsAvailable(ptrChunkModeActive) || !IsWritable(ptrChunkModeActive))
		{
			cout << "Unable to activate chunk mode. Aborting..." << endl << endl;
			return -1;
		}

		ptrChunkModeActive->SetValue(true);

		cout << "Chunk mode activated..." << endl;

		//
		// Enable all types of chunk data
		//
		// *** NOTES ***
		// Enabling chunk data requires working with nodes: "ChunkSelector"
		// is an enumeration selector node and "ChunkEnable" is a boolean. It
		// requires retrieving the selector node (which is of enumeration node 
		// type), selecting the entry of the chunk data to be enabled, retrieving 
		// the corresponding boolean, and setting it to true. 
		//
		// In this example, all chunk data is enabled, so these steps are 
		// performed in a loop. Once this is complete, chunk mode still needs to
		// be activated.
		//
		NodeList_t entries;

		// Retrieve the selector node
		CEnumerationPtr ptrChunkSelector = nodeMap.GetNode("ChunkSelector");

		if (!IsAvailable(ptrChunkSelector) || !IsReadable(ptrChunkSelector))
		{
			cout << "Unable to retrieve chunk selector. Aborting..." << endl << endl;
			return -1;
		}

		// Retrieve entries
		ptrChunkSelector->GetEntries(entries);

		cout << "Enabling entries..." << endl;

		for (int i = 0; i < entries.size(); i++)
		{
			// Select entry to be enabled
			CEnumEntryPtr ptrChunkSelectorEntry = entries.at(i);

			// Go to next node if problem occurs
			if (!IsAvailable(ptrChunkSelectorEntry) || !IsReadable(ptrChunkSelectorEntry))
			{
				continue;
			}

			ptrChunkSelector->SetIntValue(ptrChunkSelectorEntry->GetValue());

			cout << "\t" << ptrChunkSelectorEntry->GetSymbolic() << ": ";

			// Retrieve corresponding boolean
			CBooleanPtr ptrChunkEnable = nodeMap.GetNode("ChunkEnable");

			// Enable the boolean, thus enabling the corresponding chunk data
			if (!IsAvailable(ptrChunkEnable))
			{
				cout << "not available" << endl;
				result = -1;
			}
			else if (ptrChunkEnable->GetValue())
			{
				cout << "enabled" << endl;
			}
			else if (IsWritable(ptrChunkEnable))
			{
				ptrChunkEnable->SetValue(true);
				cout << "enabled" << endl;
			}
			else
			{
				cout << "not writable" << endl;
				result = -1;
			}
		}
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}



// This function configures the camera to use a trigger. First, trigger mode is 
// set to off in order to select the trigger source. Once the trigger source
// has been selected, trigger mode is then enabled, which has the camera 
// capture only a single image upon the execution of the chosen trigger.
int ConfigureTrigger(INodeMap & nodeMap, bool is_primary)
{
	int result = 0;

	cout << endl << endl << "*** CONFIGURING TRIGGER ***" << endl << endl;

	try
	{

		//
		// Ensure trigger mode off
		//
		// *** NOTES ***
		// The trigger must be disabled in order to configure whether the source
		// is software or hardware.
		//
		CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
		if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
		{
			cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
			return -1;
		}

		CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
		if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
		{
			cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

		cout << "Trigger mode disabled..." << endl;

		// If primary camra
		if (is_primary) {
			// Config the digital IO control
			CEnumerationPtr ptrLineSelector = nodeMap.GetNode("LineSelector");
			if (!IsAvailable(ptrLineSelector) || !IsWritable(ptrLineSelector)) {
				cout << "Unable to set line selector (node retriecal). Aborting" << endl;
				return -1;
			}
			CEnumEntryPtr ptrLineSelectorLine2 = ptrLineSelector->GetEntryByName("Line2");
			if (!IsAvailable(ptrLineSelectorLine2) || !IsReadable(ptrLineSelectorLine2)) {
				cout << "Unable to set line selector (enum entry retrieval). Aborting" << endl;
				return -1;
			}
			ptrLineSelector->SetIntValue(ptrLineSelectorLine2->GetValue());

			cout << "Digital IO Control line selection select Line2" << endl;
		}

		//
		// Select trigger source
		//
		// *** NOTES ***
		CEnumerationPtr ptrTriggerSource = nodeMap.GetNode("TriggerSource");
		if (!IsAvailable(ptrTriggerSource) || !IsWritable(ptrTriggerSource))
		{
			cout << "Unable to set trigger mode (node retrieval). Aborting..." << endl;
			return -1;
		}

		if (is_primary)
		{
			// Set trigger mode to software
			CEnumEntryPtr ptrTriggerSourceSoftware = ptrTriggerSource->GetEntryByName("Software");
			if (!IsAvailable(ptrTriggerSourceSoftware) || !IsReadable(ptrTriggerSourceSoftware))
			{
				cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}

			ptrTriggerSource->SetIntValue(ptrTriggerSourceSoftware->GetValue());

			cout << "Trigger source set to software..." << endl;
		}
		else
		{
			// Set trigger mode to hardware ('Line3')
			CEnumEntryPtr ptrTriggerSourceHardware = ptrTriggerSource->GetEntryByName("Line3");
			if (!IsAvailable(ptrTriggerSourceHardware) || !IsReadable(ptrTriggerSourceHardware))
			{
				cout << "Unable to set trigger mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}

			ptrTriggerSource->SetIntValue(ptrTriggerSourceHardware->GetValue());
			cout << "Trigger source set to Line 3..." << endl;

			// Set trigger overlap to read out
			CEnumerationPtr ptrTiggerOverlap = nodeMap.GetNode("TriggerOverlap");
			if (!IsAvailable(ptrTiggerOverlap) || !IsReadable(ptrTiggerOverlap))
			{
				cout << "Unable to set trigger overlap (mode retrieval). Aborting..." << endl;
				return -1;
			}
			CEnumEntryPtr ptrTiggerOverlapReadOut = ptrTiggerOverlap->GetEntryByName("ReadOut");
			if (!IsAvailable(ptrTiggerOverlapReadOut) || !IsReadable(ptrTiggerOverlapReadOut))
			{
				cout << "Unable to set trigger overlap (enum entry retrieval). Aborting..." << endl;
				return -1;
			}

			ptrTiggerOverlap->SetIntValue(ptrTiggerOverlapReadOut->GetValue());
			cout << "Trigger overlap set to Readout..." << endl;

		}

		//
		// Turn trigger mode on
		//
		// *** LATER ***
		// Once the appropriate trigger source has been set, turn trigger mode 
		// on in order to retrieve images using the trigger.
		//

		CEnumEntryPtr ptrTriggerModeOn = ptrTriggerMode->GetEntryByName("On");
		if (!IsAvailable(ptrTriggerModeOn) || !IsReadable(ptrTriggerModeOn))
		{
			cout << "Unable to enable trigger mode (enum entry retrieval). Aborting..." << endl;
			return -1;
		}

		ptrTriggerMode->SetIntValue(ptrTriggerModeOn->GetValue());

		// TODO: Blackfly and Flea3 GEV cameras need 1 second delay after trigger mode is turned on 

		cout << "Trigger mode turned back on..." << endl << endl;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

// This function retrieves a single image using the trigger. In this example, 
// only a single image is captured and made available for acquisition - as such,
// attempting to acquire two images for a single trigger execution would cause 
// the example to hang. This is different from other examples, whereby a 
// constant stream of images are being captured and made available for image
// acquisition.
/*
int GrabNextImageByTrigger(INodeMap & nodeMap, CameraPtr pCam, bool is_primary)
{
int result = 0;

try
{
//
// Use trigger to capture image
//
// *** NOTES ***
// The software trigger only feigns being executed by the Enter key;
// what might not be immediately apparent is that there is not a
// continuous stream of images being captured; in other examples that
// acquire images, the camera captures a continuous stream of images.
// When an image is retrieved, it is plucked from the stream.
//
if (is_primary)
{
// Get user input
cout << "Press the Enter key to initiate software trigger." << endl;
getchar();

// Execute software trigger
CCommandPtr ptrSoftwareTriggerCommand = nodeMap.GetNode("TriggerSoftware");
if (!IsAvailable(ptrSoftwareTriggerCommand) || !IsWritable(ptrSoftwareTriggerCommand))
{
cout << "Unable to execute trigger. Aborting..." << endl;
return -1;
}

ptrSoftwareTriggerCommand->Execute();

// TODO: Blackfly and Flea3 GEV cameras need 2 second delay after software trigger
}
else
{
// Execute hardware trigger
cout << "Use the hardware to trigger image acquisition." << endl;
}
}
catch (Spinnaker::Exception &e)
{
cout << "Error: " << e.what() << endl;
result = -1;
}

return result;
}
*/


// This function configures a number of settings on the camera including offsets 
// X and Y, width, height, and pixel format. These settings must be applied before
// BeginAcquisition() is called; otherwise, they will be read only. Also, it is
// important to note that settings are applied immediately. This means if you plan
// to reduce the width and move the x offset accordingly, you need to apply such
// changes in the appropriate order.
int ConfigureCustomImageSettings(INodeMap & nodeMap)
{
	int result = 0;

	cout << endl << endl << "*** CONFIGURING CUSTOM IMAGE SETTINGS ***" << endl << endl;

	try
	{
		//
		// Apply mono 8 pixel format
		//
		// *** NOTES ***
		// Enumeration nodes are slightly more complicated to set than other
		// nodes. This is because setting an enumeration node requires working
		// with two nodes instead of the usual one. 
		//
		// As such, there are a number of steps to setting an enumeration node: 
		// retrieve the enumeration node from the nodemap, retrieve the desired 
		// entry node from the enumeration node, retrieve the integer value from 
		// the entry node, and set the new value of the enumeration node with
		// the integer value from the entry node.
		//
		// Retrieve the enumeration node from the nodemap
		CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
		if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
		{
			// Retrieve the desired entry node from the enumeration node
			CEnumEntryPtr ptrPixelFormatSelect = ptrPixelFormat->GetEntryByName(grabPixelFormatName); // Mono8
			if (IsAvailable(ptrPixelFormatSelect) && IsReadable(ptrPixelFormatSelect))
			{
				// Retrieve the integer value from the entry node
				int64_t pixelFormatSelect = ptrPixelFormatSelect->GetValue();

				// Set integer as new value for enumeration node
				ptrPixelFormat->SetIntValue(pixelFormatSelect);

				cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
			}
			else
			{
				cout << "Pixel specific format not available..." << endl;
			}
		}
		else
		{
			cout << "Pixel format not available..." << endl;
		}

		//==========================================================================
		// Apply minimum to offset X
		//
		// *** NOTES ***
		// Numeric nodes have both a minimum and maximum. A minimum is retrieved
		// with the method GetMin(). Sometimes it can be important to check 
		// minimums to ensure that your desired value is within range.
		//
		/*
		CIntegerPtr ptrOffsetX = nodeMap.GetNode("OffsetX");
		if (IsAvailable(ptrOffsetX) && IsWritable(ptrOffsetX))
		{
		ptrOffsetX->SetValue(ptrOffsetX->GetMin());
		cout << "Offset X set to " << ptrOffsetX->GetMin() << "..." << endl;
		}
		else
		{
		cout << "Offset X not available..." << endl;
		}
		*/

		//==========================================================================
		// Apply minimum to offset Y
		// 
		// *** NOTES ***
		// It is often desirable to check the increment as well. The increment
		// is a number of which a desired value must be a multiple of. Certain
		// nodes, such as those corresponding to offsets X and Y, have an
		// increment of 1, which basically means that any value within range
		// is appropriate. The increment is retrieved with the method GetInc().
		//

		/*
		CIntegerPtr ptrOffsetY = nodeMap.GetNode("OffsetY");
		if (IsAvailable(ptrOffsetY) && IsWritable(ptrOffsetY))
		{
		ptrOffsetY->SetValue(ptrOffsetY->GetMin());
		cout << "Offset Y set to " << ptrOffsetY->GetValue() << "..." << endl;
		}
		else
		{
		cout << "Offset Y not available..." << endl;
		}
		*/

		//==========================================================================
		// Set maximum width
		//
		// *** NOTES ***
		// Other nodes, such as those corresponding to image width and height, 
		// might have an increment other than 1. In these cases, it can be
		// important to check that the desired value is a multiple of the
		// increment. However, as these values are being set to the maximum,
		// there is no reason to check against the increment.
		//


		CIntegerPtr ptrWidth = nodeMap.GetNode("Width");
		if (IsAvailable(ptrWidth) && IsWritable(ptrWidth))
		{
			int64_t widthToSet = ptrWidth->GetMax();

			ptrWidth->SetValue(widthToSet);

			cout << "Width set to " << ptrWidth->GetValue() << "..." << endl;
		}
		else
		{
			cout << "Width not available..." << endl;
		}


		//==========================================================================
		// Set maximum height
		//
		// *** NOTES ***
		// A maximum is retrieved with the method GetMax(). A node's minimum and
		// maximum should always be a multiple of its increment.
		//


		CIntegerPtr ptrHeight = nodeMap.GetNode("Height");
		if (IsAvailable(ptrHeight) && IsWritable(ptrHeight))
		{
			int64_t heightToSet = ptrHeight->GetMax();

			ptrHeight->SetValue(heightToSet);

			cout << "Height set to " << ptrHeight->GetValue() << "..." << endl << endl;
		}
		else
		{
			cout << "Height not available..." << endl << endl;
		}

	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}


// Configure Video Settings
int ConfigureVideoAndOpen(SpinVideo & video, INodeMap & nodeMap, INodeMap & nodeMapTLDevice, string outputFolder)
{
	int result = 0;

	cout << endl << endl << "*** CREATING VIDEO ***" << endl << endl;

	try
	{
		// Retrieve device serial number for filename
		string deviceSerialNumber = "";

		CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
		if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
		{
			deviceSerialNumber = ptrStringSerial->GetValue();

			cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
		}

		//
		// Get the current frame rate; acquisition frame rate recorded in hertz
		//
		// *** NOTES ***
		// The video frame rate can be set to anything; however, in order to
		// have videos play in real-time, the acquisition frame rate can be
		// retrieved from the camera.
		//
		CFloatPtr ptrAcquisitionFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
		if (!IsAvailable(ptrAcquisitionFrameRate) || !IsReadable(ptrAcquisitionFrameRate))
		{
			cout << "Unable to retrieve frame rate. Aborting..." << endl << endl;
			return -1;
		}

		float frameRateToSet = static_cast<float>(ptrAcquisitionFrameRate->GetValue());

		cout << "Frame rate to be set to " << frameRateToSet << "..." << endl;

		//==========================================================================
		// Create a unique filename
		//
		// *** NOTES ***
		// This example creates filenames according to the type of video
		// being created. Notice that '.avi' does not need to be appended to the
		// name of the file. This is because the SpinVideo object takes care
		// of the file extension automatically.
		//
		string videoFilename = outputFolder + "\\";

		switch (chosenVideoType)
		{
		case UNCOMPRESSED:
			videoFilename += "SaveToAvi-Uncompressed";
			if (deviceSerialNumber != "")
			{
				videoFilename = videoFilename + "-" + deviceSerialNumber.c_str();
			}

			break;

		case MJPG:
			videoFilename += "SaveToAvi-MJPG";
			if (deviceSerialNumber != "")
			{
				videoFilename = videoFilename + "-" + deviceSerialNumber.c_str();
			}

			break;

		case H264:
			videoFilename += "SaveToAvi-H264";
			if (deviceSerialNumber != "")
			{
				videoFilename = videoFilename + "-" + deviceSerialNumber.c_str();
			}
		}

		//==========================================================================
		// Select option and open video file type
		//
		// *** NOTES ***
		// Depending on the file type, a number of settings need to be set in
		// an object called an option. An uncompressed option only needs to
		// have the video frame rate set whereas videos with MJPG or H264
		// compressions should have more values set.
		//
		// Once the desired option object is configured, open the video file
		// with the option in order to create the video file.
		//
		// *** LATER ***
		// Once all images have been added, it is important to close the file -
		// this is similar to many other standard file streams.
		//
		// SpinVideo video;

		// Set maximum video file size to 2GB.
		// A new video file is generated when 2GB
		// limit is reached. Setting maximum file
		// size to 0 indicates no limit.
		const unsigned int k_videoFileSize = 2048;

		video.SetMaximumFileSize(k_videoFileSize);

		if (chosenVideoType == UNCOMPRESSED)
		{
			Video::AVIOption option;

			option.frameRate = frameRateToSet;

			video.Open(videoFilename.c_str(), option);
		}
		else if (chosenVideoType == MJPG)
		{
			Video::MJPGOption option;

			option.frameRate = frameRateToSet;
			option.quality = 75;

			video.Open(videoFilename.c_str(), option);
		}
		else if (chosenVideoType == H264)
		{
			Video::H264Option option;

			option.frameRate = frameRateToSet;
			option.bitrate = 1000000;
			option.height = static_cast<unsigned int>(imageHeight);
			option.width = static_cast<unsigned int>(imageWidth);

			video.Open(videoFilename.c_str(), option);
		}

	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}



// This function prepares, saves, and cleans up an video from a vector of images.
int SaveVectorToVideo(INodeMap & nodeMap, INodeMap & nodeMapTLDevice, vector<ImagePtr> & images, unsigned int id)
{
	int result = 0;

	cout << endl << endl << "*** CREATING VIDEO ***" << endl << endl;

	try
	{
		// Retrieve device serial number for filename
		string deviceSerialNumber = "";

		CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
		if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
		{
			deviceSerialNumber = ptrStringSerial->GetValue();

			cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
		}

		//
		// Get the current frame rate; acquisition frame rate recorded in hertz
		//
		// *** NOTES ***
		// The video frame rate can be set to anything; however, in order to
		// have videos play in real-time, the acquisition frame rate can be
		// retrieved from the camera.
		//
		CFloatPtr ptrAcquisitionFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
		if (!IsAvailable(ptrAcquisitionFrameRate) || !IsReadable(ptrAcquisitionFrameRate))
		{
			cout << "Unable to retrieve frame rate. Aborting..." << endl << endl;
			return -1;
		}

		float frameRateToSet = static_cast<float>(ptrAcquisitionFrameRate->GetValue());

		cout << "Frame rate to be set to " << frameRateToSet << "..." << endl;

		//==========================================================================
		// Create a unique filename
		//
		// *** NOTES ***
		// This example creates filenames according to the type of video
		// being created. Notice that '.avi' does not need to be appended to the
		// name of the file. This is because the SpinVideo object takes care
		// of the file extension automatically.
		//
		string videoFilename;

		char buffer[256]; sprintf(buffer, "%03d", id);
		string video_id(buffer);

		switch (chosenVideoType)
		{
		case UNCOMPRESSED:
			videoFilename = "SaveToAvi-Uncompressed";
			if (deviceSerialNumber != "")
			{
				videoFilename = videoFilename + "-" + deviceSerialNumber.c_str() + "-" + video_id;
			}

			break;

		case MJPG:
			videoFilename = "SaveToAvi-MJPG";
			if (deviceSerialNumber != "")
			{
				videoFilename = videoFilename + "-" + deviceSerialNumber.c_str() + "-" + video_id;
			}

			break;

		case H264:
			videoFilename = "SaveToAvi-H264";
			if (deviceSerialNumber != "")
			{
				videoFilename = videoFilename + "-" + deviceSerialNumber.c_str() + "-" + video_id;
			}
		}

		//==========================================================================
		// Select option and open video file type
		//
		// *** NOTES ***
		// Depending on the file type, a number of settings need to be set in
		// an object called an option. An uncompressed option only needs to
		// have the video frame rate set whereas videos with MJPG or H264
		// compressions should have more values set.
		//
		// Once the desired option object is configured, open the video file
		// with the option in order to create the video file.
		//
		// *** LATER ***
		// Once all images have been added, it is important to close the file -
		// this is similar to many other standard file streams.
		//
		SpinVideo video;

		// Set maximum video file size to 2GB.
		// A new video file is generated when 2GB
		// limit is reached. Setting maximum file
		// size to 0 indicates no limit.
		const unsigned int k_videoFileSize = 2048;

		video.SetMaximumFileSize(k_videoFileSize);

		if (chosenVideoType == UNCOMPRESSED)
		{
			Video::AVIOption option;

			option.frameRate = frameRateToSet;

			video.Open(videoFilename.c_str(), option);
		}
		else if (chosenVideoType == MJPG)
		{
			Video::MJPGOption option;

			option.frameRate = frameRateToSet;
			option.quality = 75;

			video.Open(videoFilename.c_str(), option);
		}
		else if (chosenVideoType == H264)
		{
			Video::H264Option option;

			option.frameRate = frameRateToSet;
			option.bitrate = 1000000;
			option.height = static_cast<unsigned int>(images[0]->GetHeight());
			option.width = static_cast<unsigned int>(images[0]->GetWidth());

			video.Open(videoFilename.c_str(), option);
		}

		//==========================================================================
		// Construct and save video
		//
		// *** NOTES ***
		// Although the video file has been opened, images must be individually
		// appended in order to construct the video.
		//
		cout << "Appending " << images.size() << " images to video file: " << videoFilename << ".avi... " << endl << endl;

		for (unsigned int imageCnt = 0; imageCnt < images.size(); imageCnt++)
		{
			video.Append(images[imageCnt]);
			// cout << "\tAppended image " << imageCnt << "..." << endl;
		}

		//==========================================================================
		// Close video file
		//
		// *** NOTES ***
		// Once all images have been appended, it is important to close the
		// video file. Notice that once an video file has been closed, no more
		// images can be added.
		//
		video.Close();

		cout << endl << "Video saved at " << videoFilename << ".avi" << endl << endl;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}


// This struct is design for run the thread function SaveVectorToVideoThread
struct SaveVectorToVideoParam {
	CameraPtr pCam;
	vector<ImagePtr> images;
	unsigned int id; // video id if take several videos

	SaveVectorToVideoParam(CameraPtr _pCam, vector<ImagePtr> _images, unsigned int _id) :
		pCam(_pCam), images(_images), id(_id) {}
};


// This function acquires and saves images from a camera.  
// only takes care of the WIN32 env
DWORD WINAPI SaveVectorToVideoThread(LPVOID lpParam)
{
	SaveVectorToVideoParam param = *((SaveVectorToVideoParam*)lpParam);
	CameraPtr pCam = param.pCam;

	try
	{
		INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
		INodeMap & nodeMap = pCam->GetNodeMap();

		int result = SaveVectorToVideo(nodeMap, nodeMap, param.images, param.id);
		if (result < 0) {
			cout << "Failed to save the images to AVI" << endl;
			return result;
		}

		return 1;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		return 0;
	}
}


// This struct is design for run the thread function SaveImageThread
struct SaveImageParam {
	ImagePtr img;
	string name;

	SaveImageParam() {
		img = Image::Create();
	}

	void releaseImg() {
		img->Release();
	}
};


// This function save images use one thread
DWORD WINAPI SaveImageThread(LPVOID lpParam)
{
	SaveImageParam* pParam = (SaveImageParam*)lpParam;

	try
	{
		pParam->img->Save(pParam->name.c_str());
		// cout << "Saving image " << pParam->name << endl;
		return 1;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Save Error: " << e.what() << endl;
		return 0;
	}
}


// This function acquires and saves images from a camera.  
#if defined (_WIN32)
DWORD WINAPI AcquireImages(LPVOID lpParam)
{
	CameraPtr pCam = *((CameraPtr*)lpParam);
#else
void* AcquireImages(void* arg)
{
	CameraPtr pCam = *((CameraPtr*)arg);
#endif
	int err = 0;
	int result = 0;

	try
	{
		// ===========================================================================================================
		// Retrieve TL device nodemap
		INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

		// Retrieve device serial number for filename
		CStringPtr ptrStringSerial = pCam->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");

		std::string serialNumber = "";
		if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
		{
			serialNumber = ptrStringSerial->GetValue();
		}

		cout << endl << "[" << serialNumber << "] " << "*** IMAGE ACQUISITION THREAD STARTING" << " ***" << endl << endl;

		bool is_primary = (serialNumber == serialNumberPrimary);

		// Print device information
		PrintDeviceInfo(nodeMapTLDevice, serialNumber);

		// ===========================================================================================================
		// Initialize camera
		pCam->Init();

		// Configure custom image settings
		err = ConfigureCustomImageSettings(pCam->GetNodeMap());
		if (err < 0) return err;

		// Configure chuck data setting
		err = ConfigureChunkData(pCam->GetNodeMap());
		// pCam->TimestampReset();
		if (err < 0) return err;

		// Configure Buffer
		err = ConfigureBuffer(pCam->GetTLStreamNodeMap());
		if (err < 0) return err;

		// Change Camera settings
		pCam->AcquisitionFrameRateEnable = true;
		pCam->AcquisitionFrameRate = selectFrameRate;

		// ===========================================================================================================
		// Configure trigger

		err = ConfigureTrigger(pCam->GetNodeMap(), is_primary);
		if (err < 0) return err;

#ifdef _DEBUG
		cout << endl << endl << "*** DEBUG ***" << endl << endl;

		// If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
		if (DisableHeartbeat(pCam, pCam->GetNodeMap(), pCam->GetTLDeviceNodeMap()) != 0)
		{
#if defined (_WIN32)
			return 0;
#else
			return (void*)0;
#endif
		}

		cout << endl << endl << "*** END OF DEBUG ***" << endl << endl;
#endif

		// ===========================================================================================================
		// Set acquisition mode to continuous
		CEnumerationPtr ptrAcquisitionMode = pCam->GetNodeMap().GetNode("AcquisitionMode");
		if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
		{
			cout << "Unable to set acquisition mode to continuous (node retrieval; camera " << serialNumber << "). Aborting..." << endl << endl;
#if defined (_WIN32)
			return 0;
#else
			return (void*)0;
#endif
		}

		CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
		if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
		{
			cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval " << serialNumber << "). Aborting..." << endl << endl;
#if defined (_WIN32)
			return 0;
#else
			return (void*)0;
#endif
		}

		int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
		ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

		cout << "[" << serialNumber << "] " << "Acquisition mode set to continuous..." << endl;

		//
		int camId = 0;
		for (int idx = 0; idx < k_numCameras; ++idx) {
			if (serialNumber == serialNumbers[idx]) camId = idx;
		}
		string outputFolder = outputFolders[camId] + "\\" + subfolderName + "\\";

		if (CreateDirectoryA(outputFolder.c_str(), NULL) ||
			ERROR_ALREADY_EXISTS == GetLastError())
			cout << "[" << serialNumber << "] " << "Output at path: " << outputFolder << endl;

		//=================================================================================
		// Init and open Video
		SpinVideo video;
		result = ConfigureVideoAndOpen(video, pCam->GetNodeMap(), nodeMapTLDevice, outputFolder);

		//=================================================================================
		// Open log file
		ofstream logFile;
		logFile.open(outputFolder + "Log" + serialNumber + ".txt");


		//=================================================================================
		// Begin acquiring images
		pCam->BeginAcquisition();

		cout << "[" << serialNumber << "] " << "Started acquiring images..." << endl;

		//==================================================================================
		// Trigger the primary camera
		if (is_primary) {
			cout << "Press Enter to start Capture. Wait about 1-2 Second to make sure  that all configurations are finished" << endl;
			cin.get();

			CEnumerationPtr ptrTriggerMode = pCam->GetNodeMap().GetNode("TriggerMode");
			if (!IsAvailable(ptrTriggerMode) || !IsReadable(ptrTriggerMode))
			{
				cout << "Unable to disable trigger mode (node retrieval). Aborting..." << endl;
				return -1;
			}

			CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
			if (!IsAvailable(ptrTriggerModeOff) || !IsReadable(ptrTriggerModeOff))
			{
				cout << "Unable to disable trigger mode (enum entry retrieval). Aborting..." << endl;
				return -1;
			}

			ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

			cout << "Trigger mode disabled... And Start Capture" << endl;
		}


		//==================================================================================
		// Retrieve, convert, and save images for each camera

		cout << endl;

		for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
		{
			try
			{
				// Retrieve next received image and ensure image completion
				ImagePtr pResultImage = pCam->GetNextImage();

				if (pResultImage->IsIncomplete())
				{
					cout << "[" << serialNumber << "] " << "Image incomplete with image status " << pResultImage->GetImageStatus() << "..." << endl << endl;
				}
				else
				{
					// ImagePtr convertedImage = pResultImage->Convert(savePixelFormat, HQ_LINEAR);
					ImagePtr convertedImage = pResultImage;

					//=============================
					// Save to an image file
					/*
					// Create a unique filename
					string filename = "imgs/";

					if (serialNumber != "") {
					filename += serialNumber.c_str();
					}

					char buffer[256]; sprintf(buffer, "%06d", imageCnt);
					string img_id(buffer);
					filename +=  "/img_" +  img_id +  ".jpg";
					*/
					// Save image
					// convertedImage->Save(filename.str().c_str());

					// Append image to video
					video.Append(convertedImage);

					result = DisplayChunkData(pResultImage, logFile, imageCnt);

					// Print image information
					if ((imageCnt + 1) % k_numPrintInfo == 0)
						cout << "[" << serialNumber << "] " << "Grabbed image " << imageCnt << ", width = " << pResultImage->GetWidth() << ", height = " << pResultImage->GetHeight() << endl; //". Image saved at " << filename.str() << endl;


				}


			}
			catch (Spinnaker::Exception &e)
			{
				// End acquisition
				pCam->EndAcquisition();
				// Deinitialize camera
				pCam->DeInit();

				video.Close();
				logFile.close();

				cout << "[" << serialNumber << "] " << "Error: " << e.what() << endl;
			}
		}

		// End acquisition
		pCam->EndAcquisition();
		
		err = DisableChunkData(pCam->GetNodeMap());
		if (err < 0) return err;

		// Deinitialize camera
		pCam->DeInit();

		video.Close();
		logFile.close();



#if defined (_WIN32)
		return 1;
#else
		return (void*)1;
#endif
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
#if defined (_WIN32)
		return 0;
#else
		return (void*)0;
#endif
	}
}


// This function acts as the body of the example
int RunMultipleCameras(CameraList camList)
{
	int result = 0;
	unsigned int camListSize = 0;

	try
	{
		// Retrieve camera list size
		camListSize = camList.GetSize();

		// Create an array of CameraPtrs. This array maintenances smart pointer's reference
		// count when CameraPtr is passed into grab thread as void pointer

		// Create an array of handles
		CameraPtr* pCamList = new CameraPtr[camListSize];
#if defined(_WIN32)
		HANDLE* grabThreads = new HANDLE[camListSize];
#else
		pthread_t* grabThreads = new pthread_t[camListSize];
#endif

		for (unsigned int i = 0; i < camListSize; i++)
		{
			// Select camera
			pCamList[i] = camList.GetByIndex(i);
			// Start grab thread
#if defined(_WIN32)
			grabThreads[i] = CreateThread(NULL, 0, AcquireImages, &pCamList[i], 0, NULL);
			assert(grabThreads[i] != NULL);
#else
			int err = pthread_create(&(grabThreads[i]), NULL, &AcquireImages, &pCamList[i]);
			assert(err == 0);
#endif
		}


#if defined(_WIN32)
		// Wait for all threads to finish
		WaitForMultipleObjects(camListSize,		// number of threads to wait for 
			grabThreads,				// handles for threads to wait for
			TRUE,					// wait for all of the threads
			INFINITE				// wait forever
		);

		// Check thread return code for each camera
		for (unsigned int i = 0; i < camListSize; i++)
		{
			DWORD exitcode;

			BOOL rc = GetExitCodeThread(grabThreads[i], &exitcode);
			if (!rc)
			{
				cout << "Handle error from GetExitCodeThread() returned for camera at index " << i << endl;
			}
			else if (!exitcode)
			{
				cout << "Grab thread for camera at index " << i << " exited with errors."
					"Please check onscreen print outs for error details" << endl;
			}
		}

#else
		for (unsigned int i = 0; i < camListSize; i++)
		{
			// Wait for all threads to finish
			void* exitcode;
			int rc = pthread_join(grabThreads[i], &exitcode);
			if (rc != 0)
			{
				cout << "Handle error from pthread_join returned for camera at index " << i << endl;
			}
			else if ((int)(intptr_t)exitcode == 0)// check thread return code for each camera
			{
				cout << "Grab thread for camera at index " << i << " exited with errors."
					"Please check onscreen print outs for error details" << endl;
			}
		}
#endif

		// Clear CameraPtr array and close all handles
		for (unsigned int i = 0; i < camListSize; i++)
		{
			pCamList[i] = 0;
#if defined(_WIN32)            
			CloseHandle(grabThreads[i]);
#endif
		}

		// Delete array pointer
		delete[] pCamList;

		// Delete array pointer
		delete[] grabThreads;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}


// Example entry point; please see Enumeration example for more in-depth 
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{
	// Since this application saves images in the current folder
	// we must ensure that we have permission to write to this folder.
	// If we do not have permission, fail right away.
	FILE *tempFile = fopen("test.txt", "w+");
	if (tempFile == NULL)
	{
		cout << "Failed to create file in current folder.  Please check permissions." << endl;
		cout << "Press Enter to exit..." << endl;
		getchar();
		return -1;
	}

	fclose(tempFile);
	remove("test.txt");

	int result = 0;

	// Print application build information
	cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

	// Retrieve singleton reference to system object
	SystemPtr system = System::GetInstance();

	// Print out current library version
	const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
	cout << "Spinnaker library version: "
		<< spinnakerLibraryVersion.major << "."
		<< spinnakerLibraryVersion.minor << "."
		<< spinnakerLibraryVersion.type << "."
		<< spinnakerLibraryVersion.build << endl << endl;

	// Retrieve list of cameras from the system
	CameraList camList = system->GetCameras();

	unsigned int numCameras = camList.GetSize();

	cout << "Number of cameras detected: " << numCameras << endl << endl;

	// Finish if there are no cameras
	if (numCameras < k_numCameras)
	{
		// Clear camera list before releasing system
		camList.Clear();

		// Release system
		system->ReleaseInstance();

		cout << "Not enough cameras!" << endl;
		cout << "Done! Press Enter to exit..." << endl;
		getchar();

		return -1;
	}

	// Run example on all cameras
	cout << endl << "Running example for all cameras..." << endl;

	result = RunMultipleCameras(camList);

	cout << "Example complete..." << endl << endl;

	// Clear camera list before releasing system
	camList.Clear();

	// Release system
	system->ReleaseInstance();

	cout << endl << "Done! Press Enter to exit..." << endl;
	getchar();

	return result;
}


//winsat disk -drive f
//Windows System Assessment Tool
//> Running: Feature Enumeration ''
//> Run Time 00:00 : 00.00
//> Running: Storage Assessment '-drive f -ran -read'
//> Run Time 00:00 : 07.95
//> Running: Storage Assessment '-drive f -seq -read'
//> Run Time 00:00 : 06.36
//> Running: Storage Assessment '-drive f -seq -write'
//> Run Time 00:00 : 05.23
//> Running: Storage Assessment '-drive f -flush -seq'
//> Run Time 00:00 : 03.47
//> Running: Storage Assessment '-drive f -flush -ran'
//> Run Time 00:00 : 06.83
//> Dshow Video Encode Time                      0.00000 s
//> Dshow Video Decode Time                      0.00000 s
//> Media Foundation Decode Time                 0.00000 s
//> Disk  Random 16.0 Read                       2.16 MB / s          4.3
//> Disk  Sequential 64.0 Read                   141.53 MB / s          7.1
//> Disk  Sequential 64.0 Write                  172.27 MB / s          7.2
//> Average Read Time with Sequential Writes     2.965 ms          6.7
//> Latency: 95th Percentile                     16.685 ms          4.9
//> Latency: Maximum                             93.788 ms          7.7
//> Average Read Time with Random Writes         6.035 ms          5.7
//> Total Run Time 00:00 : 30.55