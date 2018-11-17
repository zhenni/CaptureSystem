#=============================================================================
# Copyright 2017 FLIR Integrated Imaging Solutions, Inc. All Rights Reserved.
#
# This software is the confidential and proprietary information of FLIR
# Integrated Imaging Solutions, Inc. ("Confidential Information"). You
# shall not disclose such Confidential Information and shall use it only in
# accordance with the terms of the license agreement you entered into
# with FLIR Integrated Imaging Solutions, Inc. (FLIR).
#
# FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
# SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
# PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
# SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
# THIS SOFTWARE OR ITS DERIVATIVES.
#=============================================================================

import PyCapture2
import pdb

def printBuildInfo():
	libVer = PyCapture2.getLibraryVersion()
	print "PyCapture2 library version: ", libVer[0], libVer[1], libVer[2], libVer[3]
	print

def printCameraInfo(cam):
	camInfo = cam.getCameraInfo()
	print "\n*** CAMERA INFORMATION ***\n"
	print "Serial number - ", camInfo.serialNumber
	print "Camera model - ", camInfo.modelName
	print "Camera vendor - ", camInfo.vendorName
	print "Sensor - ", camInfo.sensorInfo
	print "Resolution - ", camInfo.sensorResolution
	print "Firmware version - ", camInfo.firmwareVersion
	print "Firmware build time - ", camInfo.firmwareBuildTime
	print

def saveAviHelper(cams, fileFormat, fileName, frameRate):
	numImages = 100
	numCams = len(cams)

	avis = [PyCapture2.AVIRecorder() for i in range(numCams)]

	for i in range(numImages):
		images = []
		for camIdx in range(numCams):
			try:
				image = cams[camIdx].retrieveBuffer()
			except PyCapture2.Fc2error as fc2Err:
				print "Error retrieving buffer : ", fc2Err
				continue
			

			timestamp = image.getTimeStamp()
			print "Camera {} - Frame {} - TimeStamp [ {} {} ] ".format(camIdx, i, timestamp.cycleSeconds, timestamp.cycleCount)

			images.append(image)
			# print "Grabbed image {} for Cam {}".format(i, camIdx)

		for camIdx in range(numCams):
			if (i == 0):
				if fileFormat == "AVI":
					avis[camIdx].AVIOpen(fileName.format(camIdx), frameRate)
				elif fileFormat == "MJPG":
					avis[camIdx].MJPGOpen(fileName.format(camIdx), frameRate, 75)
				elif fileFormat == "H264":
					avis[camIdx].H264Open(fileName.format(camIdx), frameRate, image.getCols(), image.getRows(), 1000000)
				else:
					print "Specified format is not available."
					return

			
			avis[camIdx].append(images[camIdx])
			print "Appended image {} for cam {}...".format(i, camIdx)

	for camIdx in range(numCams):
		print "Appended {} images for Cam {} to {} file: {}...".format(numImages, camIdx, fileFormat, fileName.format(camIdx))
		avis[camIdx].close()

#
# Example Main
#

# Print PyCapture2 Library Information
printBuildInfo()

# Ensure sufficient cameras are found
bus = PyCapture2.BusManager()
numCams = bus.getNumOfCameras()
print "Number of cameras detected: ", numCams
if not numCams:
	print "Insufficient number of cameras. Exiting..."
	exit()

cams = [0]*numCams
for idx in range(numCams):
	# Select camera on 0th index
	cams[idx] = PyCapture2.Camera()
	cams[idx].connect(bus.getCameraFromIndex(idx))

	# Print camera details
	printCameraInfo(cams[idx])

	cams[idx].setVideoModeAndFrameRate(17, 2)

print "Starting capture..."
PyCapture2.startSyncCapture(cams)

# print "Detecting frame rate from Camera"
# fRateProp = cam.getProperty(PyCapture2.PROPERTY_TYPE.FRAME_RATE)
# frameRate = fRateProp.absValue
frameRate = 7.5
print "Using frame rate of {}".format(frameRate)

# for fileFormat in ("H264"):# ("AVI","H264","MJPG"):
fileFormat = "AVI"
print "start"
fileName = "SaveImageToAviEx_{}.avi"
saveAviHelper(cams, fileFormat, fileName, frameRate)

print "Stopping capture..."
for camIdx in range(numCams):
	cams[camIdx].stopCapture()
	cams[camIdx].disconnect()

raw_input("Done! Press Enter to exit...\n")
