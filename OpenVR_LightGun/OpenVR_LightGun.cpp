// OpenVR_LightGun.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <stdlib.h>
#include "../openvr/headers/openvr.h"
#pragma comment(lib, "../openvr/lib/win32/openvr_api.lib")
#include <windows.h>
#include <vector>
#include <iostream>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <mutex>
#include <set>

using namespace std;
using namespace vr;

set<int> available_controllers;


std::mutex end_signal_mutex;
bool end_signal=false;
std::mutex tracking_device_mutex;
float *screen_plane[2][2];
POINT cursor_position;                // previous cursor location 




void send_end_signal();
bool check_end_signal();
void run_mouse_emulation(int device_number, string com_port);

string ftos(float f, int precision);	// float to string with 2-decimal precision
string vftos(float v[3], int precision);	// float vector to string with 2-decimal precisions
string GetTrackedDeviceString(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::TrackedDeviceProperty, vr::TrackedPropertyError *peError = NULL);
string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class);


vr::IVRSystem* vr_context;
vr::TrackedDevicePose_t tracked_device_pose[vr::k_unMaxTrackedDeviceCount];

bool app_end = false;
string driver_name, driver_serial;
string tracked_device_type[vr::k_unMaxTrackedDeviceCount];

int init_OpenVR();
void process_vr_event(const vr::VREvent_t & event);
void exit();
float *get_coordinate_trigger_press(int device_number);


int main()
{
	std::cout << "OpenVR_LightGun!\n" << endl;
	std::cout << (("Driver name: " + driver_name).c_str()) << " ";

	std::cout << (("Driver serial ID: " + driver_serial).c_str()) << endl;
	if (init_OpenVR() != 0) { std::cout << "Failed to initialise OpenVR"; return -1; }
	int emulator_count = 0;
	thread emulator_threads[64];

	bool screen_coordinates_configured = false;
	

	if (vr_context != NULL)
	{

		for (;;) {
			cout << "Please select the device number you would like to use as light_gun" << endl;
			int device_number;
			cin >> device_number;
			if (available_controllers.find(device_number) == available_controllers.end()) {
				cout << "Invalid device number" << endl;
				continue;
			}

			if (!screen_coordinates_configured) {
				for (;;) {

					cout << "Please bring the controller to the top left edge of the screen and press the trigger" << endl;
					screen_plane[0][0] = get_coordinate_trigger_press(device_number);
					cout << "Please bring the controller to the top right edge of the screen and press the trigger" << endl;
					screen_plane[0][1] = get_coordinate_trigger_press(device_number);
					cout << "Please bring the controller to the bottom left edge of the screen and press the trigger" << endl;
					screen_plane[1][0] = get_coordinate_trigger_press(device_number);
					cout << "Please bring the controller to the bottom right edge of the screen and press the trigger" << endl;
					screen_plane[1][1] = get_coordinate_trigger_press(device_number);
					cout << "Press x to recalibrate, otherwise press any key to continue" << endl;
					char response;
					cin >> response;
					if (response != 'x') {
						break;
					}

				}
				screen_coordinates_configured = true;
			}

			cout << "Select enter a com port number for this light_gun(you can find this in windows device manager)" << endl;
			int com_port;
			cin >> com_port;

			//check if valid port here
			emulator_threads[emulator_count] = std::thread(run_mouse_emulation, device_number, "COM" + com_port);
			emulator_threads[emulator_count].detach();
			emulator_count += 1;


			bool cont_statement = true;
			char response;


			while (cont_statement) {
				cont_statement = false;
				cout << "would you like to add another controller? (y or n)" << endl;
				cin >> response;	
				if (response != 'y'&&response != 'n') {
					cont_statement = true;
					cout << "Invalid input" << endl;
				}
			}
			if (response == 'n') {
				break;
			}
		}
		
		for (;;) {
			cout << "mouse emulation running, to end emulation, enter \"end\" " << endl;
			string end_command;
			cin >> end_command;
			if (end_command=="end") {
				for (int i = 0; i < emulator_count; i++) {
					send_end_signal();
				}
				//break;
			}
			else {
				cout << "invalid command" << endl;
			}


		}

		return 0;
		
	}


}



int init_OpenVR()
{
	// Check whether there is an HMD plugged-in and the SteamVR runtime is installed
	if (vr::VR_IsHmdPresent())
	{
		cout << "An HMD was successfully found in the system" << endl;

		if (vr::VR_IsRuntimeInstalled()) {
			const char* runtime_path = vr::VR_RuntimePath();
			cout << "Runtime correctly installed at '" << runtime_path << "'" << endl;
		}
		else
		{
			cout << "Runtime was not found, quitting app" << endl;
			return -1;
		}
	}
	else
	{
		cout << "No HMD was found in the system, quitting app" << endl;
		return -1;
	}

	// Load the SteamVR Runtime
	vr::HmdError err;
	vr_context = vr::VR_Init(&err, vr::EVRApplicationType::VRApplication_Scene);
	vr_context == NULL ? cout << "Error while initializing SteamVR runtime. Error code is " << vr::VR_GetVRInitErrorAsSymbol(err) << endl : cout << "SteamVR runtime successfully initialized" << endl;

	// Obtain some basic information given by the runtime
	int base_stations_count = 0;
	for (uint32_t td = vr::k_unTrackedDeviceIndex_Hmd; td < vr::k_unMaxTrackedDeviceCount; td++) {

		if (vr_context->IsTrackedDeviceConnected(td))
		{
			vr::ETrackedDeviceClass tracked_device_class = vr_context->GetTrackedDeviceClass(td);

			string td_type = GetTrackedDeviceClassString(tracked_device_class);
			tracked_device_type[td] = td_type;

			if (td_type=="controller") {
				cout << "Tracking device " << td << " is connected " << endl;
				cout << "  Device type: " << td_type << ". Name: " << GetTrackedDeviceString(vr_context, td, vr::Prop_TrackingSystemName_String) << endl;
				available_controllers.insert(td);
			}

			
			if (tracked_device_class == vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference) base_stations_count++;

			if (td == vr::k_unTrackedDeviceIndex_Hmd)
			{
				// Fill variables used for obtaining the device name and serial ID (used later for naming the SDL window)
				driver_name = GetTrackedDeviceString(vr_context, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
				driver_serial = GetTrackedDeviceString(vr_context, vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);
			}
		}
	}

	// Check whether both base stations are found, not mandatory but just in case...
	if (base_stations_count < 2)
	{
		cout << "There was a problem indentifying the base stations, please check they are powered on" << endl;

		return -1;
	}

	return 0;
}


void process_vr_event(const vr::VREvent_t & event)
{
	string str_td_class = GetTrackedDeviceClassString(vr_context->GetTrackedDeviceClass(event.trackedDeviceIndex));

	switch (event.eventType)
	{
	case vr::VREvent_TrackedDeviceActivated:
	{
		cout << "Device " << event.trackedDeviceIndex << " attached (" << str_td_class << ")" << endl;
		tracked_device_type[event.trackedDeviceIndex] = str_td_class;
	}
	break;
	case vr::VREvent_TrackedDeviceDeactivated:
	{
		cout << "Device " << event.trackedDeviceIndex << " detached (" << str_td_class << ")" << endl;
		tracked_device_type[event.trackedDeviceIndex] = "";
	}
	break;
	case vr::VREvent_TrackedDeviceUpdated:
	{
		cout << "Device " << event.trackedDeviceIndex << " updated (" << str_td_class << ")" << endl;
	}
	break;
	case vr::VREvent_ButtonPress:
	{
		vr::VREvent_Controller_t controller_data = event.data.controller;
		cout << "Pressed button " << vr_context->GetButtonIdNameFromEnum((vr::EVRButtonId) controller_data.button) << " of device " << event.trackedDeviceIndex << " (" << str_td_class << ")" << endl;
	}
	break;
	case vr::VREvent_ButtonUnpress:
	{
		vr::VREvent_Controller_t controller_data = event.data.controller;
		cout << "Unpressed button " << vr_context->GetButtonIdNameFromEnum((vr::EVRButtonId) controller_data.button) << " of device " << event.trackedDeviceIndex << " (" << str_td_class << ")" << endl;
	}
	break;
	case vr::VREvent_ButtonTouch:
	{
		vr::VREvent_Controller_t controller_data = event.data.controller;
		cout << "Touched button " << vr_context->GetButtonIdNameFromEnum((vr::EVRButtonId) controller_data.button) << " of device " << event.trackedDeviceIndex << " (" << str_td_class << ")" << endl;
	}
	break;
	case vr::VREvent_ButtonUntouch:
	{
		vr::VREvent_Controller_t controller_data = event.data.controller;
		cout << "Untouched button " << vr_context->GetButtonIdNameFromEnum((vr::EVRButtonId) controller_data.button) << " of device " << event.trackedDeviceIndex << " (" << str_td_class << ")" << endl;
	}
	break;
	}
}

void exit()
{
	vr::VR_Shutdown();

}
// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file


//-----------------------------------------------------------------------------
// Purpose: helper to get a string from a tracked device property and turn it
//			into a std::string
//-----------------------------------------------------------------------------
string GetTrackedDeviceString(vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError)
{
	uint32_t requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
	if (requiredBufferLen == 0)
		return "";

	char *pchBuffer = new char[requiredBufferLen];
	requiredBufferLen = pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, requiredBufferLen, peError);
	string sResult = pchBuffer;
	delete[] pchBuffer;

	return sResult;
}

//-----------------------------------------------------------------------------
// Purpose: helper to get a string from a tracked device type class
//-----------------------------------------------------------------------------
string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class) {

	string str_td_class = "Unknown class";

	switch (td_class)
	{
	case vr::TrackedDeviceClass_Invalid:			// = 0, the ID was not valid.
		str_td_class = "invalid";
		break;
	case vr::TrackedDeviceClass_HMD:				// = 1, Head-Mounted Displays
		str_td_class = "hmd";
		break;
	case vr::TrackedDeviceClass_Controller:			// = 2, Tracked controllers
		str_td_class = "controller";
		break;
	case vr::TrackedDeviceClass_GenericTracker:		// = 3, Generic trackers, similar to controllers
		str_td_class = "generic tracker";
		break;
	case vr::TrackedDeviceClass_TrackingReference:	// = 4, Camera and base stations that serve as tracking reference points
		str_td_class = "base station";
		break;
	case vr::TrackedDeviceClass_DisplayRedirect:	// = 5, Accessories that aren't necessarily tracked themselves, but may redirect video output from other tracked devices
		str_td_class = "display redirect";
		break;
	}

	return str_td_class;
}


//-----------------------------------------------------------------------------
// Purpose: helper to convert an 3-float array into a vector string
//-----------------------------------------------------------------------------
string vftos(float v[3], int precision)
{
	stringstream stream;
	stream << "(" << ftos(v[0], precision) << "," << ftos(v[1], precision) << "," << ftos(v[2], precision) << ")";
	string f_str = stream.str();

	return f_str;
}

//-----------------------------------------------------------------------------
// Purpose: helper to convert an float into a string
//-----------------------------------------------------------------------------
string ftos(float f, int precision)
{
	stringstream stream;
	stream << fixed << setprecision(precision) << f;
	string f_str = stream.str();

	return f_str;
}



void send_end_signal() {//set boolean to end VR thread
	end_signal_mutex.lock();
	end_signal = true;
	end_signal_mutex.unlock();
}

bool check_end_signal()//check if VR thread is ended
{
	bool check = false;
	end_signal_mutex.lock();
	if (end_signal) {
		check = true;
	}
	end_signal_mutex.unlock();
	return check;
}


void run_mouse_emulation(int device_number, string com_port) {//controller device number and com ports
	bool tracking_values_available;
	float *matrix[3];

	cout << "Position you controller directly onto the cursor, pointing directly at the tip of the cursor and press trigger";
	GetCursorPos(&cursor_position);


	VREvent_t event;

	for (;;) {

		tracking_values_available = false;
		tracking_device_mutex.lock();

		
		//obtain rotation and translation matrix for device_number
		vr_context->GetDeviceToAbsoluteTrackingPose(vr::ETrackingUniverseOrigin::TrackingUniverseStanding, 0, tracked_device_pose, vr::k_unMaxTrackedDeviceCount);

		TrackedDevicePose_t trackedDevicePose;
		VRControllerState_t controller_state;
		vr_context->GetControllerStateWithPose(
			TrackingUniverseStanding, device_number, &controller_state,
			sizeof(controller_state), &trackedDevicePose);

		int trigger_index = 0;

		for (int x = 0; x < k_unControllerStateAxisCount; x++) {
			int prop = vr_context->GetInt32TrackedDeviceProperty(device_number,
				(ETrackedDeviceProperty)(Prop_Axis0Type_Int32 + x));
			if (prop == k_eControllerAxis_Trigger)
				trigger_index = x;
		}

		//cout << "Controller trigger value" << controller_state.rAxis[trigger_index].x << endl;
		for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++) {
			vr::VRControllerState_t state;
			if (vr_context->GetControllerState(unDevice, &state,sizeof(state))) {
				cout <<state.rAxis[0].x <<endl;
				cout << state.rAxis[1].x << endl;
				cout << state.rAxis[2].x << endl;
				cout << state.rAxis[3].x << endl;
				cout << state.rAxis[4].x << endl;
			}
		}

		if ((tracked_device_pose[device_number].bDeviceIsConnected) && (tracked_device_pose[device_number].bPoseIsValid))
		{
			matrix[0] = tracked_device_pose[device_number].mDeviceToAbsoluteTracking.m[0];
			matrix[1] = tracked_device_pose[device_number].mDeviceToAbsoluteTracking.m[1];
			matrix[2] = tracked_device_pose[device_number].mDeviceToAbsoluteTracking.m[2];
			tracking_values_available = true;
		}
		tracking_device_mutex.unlock();


		if (tracking_values_available) {//then send the vector to COM
			float abs_position[3] = { matrix[0][3], matrix[1][3], matrix[2][3] };// absolute position vector of device;
			//find direction vector from obtained rotation matrix
			float sqrt_magnitude = sqrt(1/3.00);
			float direction_vector[3] = { (matrix[0][0] + matrix[1][0] + matrix[2][0]) * sqrt_magnitude, (matrix[0][1] + matrix[1][1] + matrix[2][1]) * sqrt_magnitude,(matrix[0][2] + matrix[1][2] + matrix[2][2]) * sqrt_magnitude };
			float vector_magnitude = pow(direction_vector[0], 2) + pow(direction_vector[1], 2) + pow(direction_vector[2], 2);
			//cout << "device number " << device_number << " magnitude" <<vector_magnitude << endl;
			//TODO: find intersection with screen plane

			//TODO: send data to serial

		}
		Sleep(500);//expected polling rate of 200Hz
		if (check_end_signal()) {//if end signal received, end thread
			cout << "thread for device "<< device_number << " ended" << endl;
			return;
		}
	}


}


float *get_coordinate_trigger_press(int device_number) {
	float coordinates[2];




	return coordinates;
}