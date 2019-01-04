// OpenVR_LightGun.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
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
#include <stdio.h>
#include <tchar.h>
#include <math.h> 






using namespace std;
using namespace vr;

set<int> available_controllers;


std::mutex end_signal_mutex;
bool end_signal=false;
std::mutex tracking_device_mutex;
float ***screen_plane_input;

float screen_plane_adjusted[4]; //form ax,by,cz,d
float *screen_plane_corners[4];//four corners of the screen plane correspond to the screen

float screen_normal_unit_vector[3];


void send_end_signal();
bool check_end_signal();
void run_mouse_emulation(int device_number, string com_port);

string ftos(float f, int precision);	// float to string with 2-decimal precision
string vftos(float v[3], int precision);	// float vector to string with 2-decimal precisions
string GetTrackedDeviceString(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::TrackedDeviceProperty, vr::TrackedPropertyError *peError = NULL);
string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class);



float dot_product(float v1[3], float v2[3]);
void multiplyMatrices(float *firstMatrix[3], float *secondMatrix[3], float *mult[3]);
void vector_scalar_mult(float *x[3], float scalar);
void sum_matrix(float **x, float **y);//x will be updated
float **get_rotation_matrix(float f[3], float t[3]);// rotation of a into b




vr::IVRSystem* vr_context;

bool app_end = false;
string driver_name, driver_serial;
string tracked_device_type[vr::k_unMaxTrackedDeviceCount];

int init_OpenVR();
void process_vr_event(const vr::VREvent_t & event);
void exit();
float *get_coordinate_trigger_press(int device_number, float *recv_direction_vector = {});


int main()
{
	std::cout << "OpenVR_LightGun!\n" << endl;
	std::cout << (("Driver name: " + driver_name).c_str()) << " ";

	std::cout << (("Driver serial ID: " + driver_serial).c_str()) << endl;
	if (init_OpenVR() != 0) { std::cout << "Failed to initialise OpenVR"; return -1; }
	int emulator_count = 0;
	thread emulator_threads[64];

	bool screen_coordinates_configured = false;
	

	screen_plane_input = new float**[2];
	screen_plane_input[0] = new float*[2];
	screen_plane_input[1] = new float*[2];

	int device_number_arr[64];
	int com_port_arr[64];
	if (vr_context != NULL)
	{

		for (;;) {
			std::cout << "Please select the device number you would like to use as light_gun" << endl;
			int device_number;
			cin >> device_number;
			if (available_controllers.find(device_number) == available_controllers.end()) {
				std::cout << "Invalid device number" << endl;
				continue;
			}

			if (!screen_coordinates_configured) {
				for (;;) {

					std::cout << "Please bring the controller to the top left edge of the screen and press the trigger" << endl;
					screen_plane_input[0][0] = get_coordinate_trigger_press(device_number);
					
					std::cout << "Please bring the controller to the top right edge of the screen and press the trigger" << endl;
					screen_plane_input[0][1] = get_coordinate_trigger_press(device_number);
					
					std::cout << "Please bring the controller to the bottom left edge of the screen and press the trigger" << endl;
					screen_plane_input[1][0] = get_coordinate_trigger_press(device_number);

					std::cout << "Please bring the controller to the bottom right edge of the screen and press the trigger" << endl;
					screen_plane_input[1][1] = get_coordinate_trigger_press(device_number);

					std::cout << "Press x to recalibrate, otherwise press any key to continue" << endl;
					char response;
					cin >> response;
					if (response != 'x') {
						break;
					}

				}
				screen_coordinates_configured = true;
				

				float v1[3];
				v1[0] = screen_plane_input[0][0][0]- screen_plane_input[0][1][0];
				v1[1] = screen_plane_input[0][0][1] - screen_plane_input[0][1][1];
				v1[2] = screen_plane_input[0][0][2] - screen_plane_input[0][1][2];
				float v2[3];
				v2[0] = screen_plane_input[0][0][0] - screen_plane_input[1][0][0];
				v2[1] = screen_plane_input[0][0][1] - screen_plane_input[1][0][1];
				v2[2] = screen_plane_input[0][0][2] - screen_plane_input[1][0][2];
				/*
				cout << "AAA" << screen_plane_input[0][0][0] << "," << screen_plane_input[0][0][1] << "," << screen_plane_input[0][0][1] << endl;
				cout << "AAA" << screen_plane_input[0][1][0] << "," << screen_plane_input[0][1][1] << "," << screen_plane_input[0][1][2] << endl;
				cout << "AAA" << screen_plane_input[1][0][0] << "," << screen_plane_input[1][0][1] << "," << screen_plane_input[1][0][1] << endl;
				cout << "AAA" << screen_plane_input[1][1][0] << "," << screen_plane_input[1][1][1] << "," << screen_plane_input[1][1][2] << endl;
				*/
				
				screen_normal_unit_vector[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
				screen_normal_unit_vector[1] = -((v1[0] * v2[2]) - (v1[2] * v2[0]));
				screen_normal_unit_vector[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);

				float magnitude = pow((pow(screen_normal_unit_vector[0], 2) + pow(screen_normal_unit_vector[1], 2) + pow(screen_normal_unit_vector[2], 2)), 0.5);
				
				screen_normal_unit_vector[0]= screen_normal_unit_vector[0]/ magnitude;
				screen_normal_unit_vector[1]= screen_normal_unit_vector[1] / magnitude;
				screen_normal_unit_vector[2]= screen_normal_unit_vector[2] / magnitude;


				screen_plane_adjusted[0] = screen_normal_unit_vector[0];
				screen_plane_adjusted[1] = screen_normal_unit_vector[1];
				screen_plane_adjusted[2] = screen_normal_unit_vector[2];
				
				screen_plane_adjusted[3] = -(screen_normal_unit_vector[0] * screen_plane_input[0][0][0] + screen_normal_unit_vector[1] * screen_plane_input[0][0][1] + screen_normal_unit_vector[2] * screen_plane_input[0][0][2]);

				cout << screen_plane_adjusted[0] << "," << screen_plane_adjusted[1] << "," << screen_plane_adjusted[2] << "," << screen_plane_adjusted[3] << endl;


				screen_plane_corners[0] = screen_plane_input[0][0];
				screen_plane_corners[1] = screen_plane_input[0][1];
				screen_plane_corners[2] = screen_plane_input[1][0];

				float a = screen_plane_adjusted[0], b = screen_plane_adjusted[1], c = screen_plane_adjusted[2], d = screen_plane_adjusted[3];
				float p = screen_plane_input[0][0][0], q = screen_plane_input[0][0][1], r = screen_plane_input[0][0][2];
				float t = (-d - a * p - b * q - c * r) / (a*a + b * b + c * c);
				screen_plane_corners[3] = new float[3];
				screen_plane_corners[3][0]= p + a * t;
				screen_plane_corners[3][1] = q + b * t;
				screen_plane_corners[3][2] = r + c * t;
					

				//cout << "screen_plane_corners" << screen_plane_corners[3][0] << screen_plane_corners[3][1] << screen_plane_corners[3][2] << endl;

			}

			std::cout << "Enter a valid com port number for this light_gun(you can find this in windows device manager)" << endl;
			int com_port;
			cin >> com_port;

			//TODO: check if valid port here
			
			device_number_arr[emulator_count] = device_number;
			com_port_arr[emulator_count] = com_port;

			bool cont_statement = true;
			char response;


			while (cont_statement) {
				cont_statement = false;
				std::cout << "would you like to add another controller? (y or n)" << endl;
				cin >> response;	
				if (response != 'y'&&response != 'n') {
					cont_statement = true;
					std::cout << "Invalid input" << endl;
				}
			}
			
			emulator_count += 1;
			if (response == 'n') {
				break;
			}
		}


		std::cout << "mouse emulation running, to end emulation, enter \"end\" " << endl;


		for (int x = 0; x < emulator_count; x++) {
			emulator_threads[emulator_count] = std::thread(run_mouse_emulation, device_number_arr[x], "COM" + com_port_arr[x]);
			emulator_threads[emulator_count].detach();
		}

		
		for (;;) {
			string end_command;
			cin >> end_command;
			if (end_command=="end") {
				for (int i = 0; i < emulator_count; i++) {
					send_end_signal();
				}
				//break;
			}
			else {
				std::cout << "invalid command. Enter \"end\" to end emulation" << endl;
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
		std::cout << "An HMD was successfully found in the system" << endl;

		if (vr::VR_IsRuntimeInstalled()) {
			const char* runtime_path = vr::VR_RuntimePath();
			std::cout << "Runtime correctly installed at '" << runtime_path << "'" << endl;
		}
		else
		{
			std::cout << "Runtime was not found, quitting app" << endl;
			return -1;
		}
	}
	else
	{
		std::cout << "No HMD was found in the system, quitting app" << endl;
		return -1;
	}

	// Load the SteamVR Runtime
	vr::HmdError err;
	vr_context = vr::VR_Init(&err, vr::EVRApplicationType::VRApplication_Scene);
	vr_context == NULL ? std::cout << "Error while initializing SteamVR runtime. Error code is " << vr::VR_GetVRInitErrorAsSymbol(err) << endl : std::cout << "SteamVR runtime successfully initialized" << endl;

	// Obtain some basic information given by the runtime
	int base_stations_count = 0;
	for (uint32_t td = vr::k_unTrackedDeviceIndex_Hmd; td < vr::k_unMaxTrackedDeviceCount; td++) {

		if (vr_context->IsTrackedDeviceConnected(td))
		{
			vr::ETrackedDeviceClass tracked_device_class = vr_context->GetTrackedDeviceClass(td);

			string td_type = GetTrackedDeviceClassString(tracked_device_class);
			tracked_device_type[td] = td_type;

			if (td_type=="controller") {
				std::cout << "Tracking device " << td << " is connected " << endl;
				std::cout << "  Device type: " << td_type << ". Name: " << GetTrackedDeviceString(vr_context, td, vr::Prop_TrackingSystemName_String) << endl;
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
		std::cout << "There was a problem indentifying the base stations, please check they are powered on" << endl;

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
		std::cout << "Device " << event.trackedDeviceIndex << " attached (" << str_td_class << ")" << endl;
		tracked_device_type[event.trackedDeviceIndex] = str_td_class;
	}
	break;
	case vr::VREvent_TrackedDeviceDeactivated:
	{
		std::cout << "Device " << event.trackedDeviceIndex << " detached (" << str_td_class << ")" << endl;
		tracked_device_type[event.trackedDeviceIndex] = "";
	}
	break;
	case vr::VREvent_TrackedDeviceUpdated:
	{
		std::cout << "Device " << event.trackedDeviceIndex << " updated (" << str_td_class << ")" << endl;
	}
	break;
	case vr::VREvent_ButtonPress:
	{
		vr::VREvent_Controller_t controller_data = event.data.controller;
		std::cout << "Pressed button " << vr_context->GetButtonIdNameFromEnum((vr::EVRButtonId) controller_data.button) << " of device " << event.trackedDeviceIndex << " (" << str_td_class << ")" << endl;
	}
	break;
	case vr::VREvent_ButtonUnpress:
	{
		vr::VREvent_Controller_t controller_data = event.data.controller;
		std::cout << "Unpressed button " << vr_context->GetButtonIdNameFromEnum((vr::EVRButtonId) controller_data.button) << " of device " << event.trackedDeviceIndex << " (" << str_td_class << ")" << endl;
	}
	break;
	case vr::VREvent_ButtonTouch:
	{
		vr::VREvent_Controller_t controller_data = event.data.controller;
		std::cout << "Touched button " << vr_context->GetButtonIdNameFromEnum((vr::EVRButtonId) controller_data.button) << " of device " << event.trackedDeviceIndex << " (" << str_td_class << ")" << endl;
	}
	break;
	case vr::VREvent_ButtonUntouch:
	{
		vr::VREvent_Controller_t controller_data = event.data.controller;
		std::cout << "Untouched button " << vr_context->GetButtonIdNameFromEnum((vr::EVRButtonId) controller_data.button) << " of device " << event.trackedDeviceIndex << " (" << str_td_class << ")" << endl;
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
	POINT cursor_position;
	std::cout << "Position controller "+ std::to_string(device_number) +" directly onto the cursor, pointing directly at the tip of the cursor and press trigger";
	float *direction_vector_cursor_controller = new float[3]; //direction vector of controller
	float *abs_position_controller_cursor = get_coordinate_trigger_press(device_number, direction_vector_cursor_controller);//x,y,z coordinates
	
	//find rotation matrix between direction vector and screen normal vector
	float **rotation_matrix = get_rotation_matrix(direction_vector_cursor_controller, screen_normal_unit_vector);



	GetCursorPos(&cursor_position);

	//TODO: find direction vector between screen_normal_unit_vector and direction_vector_cursor_controller


	VREvent_t event;

	for (;;) {
		//button tracking only works when controller plugged in

		tracking_values_available = false;
		tracking_device_mutex.lock();//thread safety for multiple controllers
		TrackedDevicePose_t tracked_device_pose;
		VRControllerState_t controllerState;

		vr_context->GetControllerStateWithPose(//get controller state and translation
			TrackingUniverseStanding, device_number, &controllerState,
			sizeof(controllerState), &tracked_device_pose);
		tracking_device_mutex.unlock();
		
		if ((tracked_device_pose.bPoseIsValid && tracked_device_pose.bDeviceIsConnected))
		{
			matrix[0] = tracked_device_pose.mDeviceToAbsoluteTracking.m[0];
			matrix[1] = tracked_device_pose.mDeviceToAbsoluteTracking.m[1];
			matrix[2] = tracked_device_pose.mDeviceToAbsoluteTracking.m[2];
			tracking_values_available = true;
		}		
		if (tracking_values_available) {//then process controller position

			float abs_position[3] = { matrix[0][3], matrix[1][3], matrix[2][3] };// absolute position vector of device;
			//find direction vector from obtained rotation matrix
			float sqrt_magnitude = sqrt(1 / 3.00);
			std::cout << "Device " << device_number << endl << " Controller coordinates(" << abs_position[0] << "," << abs_position[1] << "," << abs_position[2] << ")" << endl;
			float direction_vector[3] = { (matrix[0][0] + matrix[1][0] + matrix[2][0]) * sqrt_magnitude, (matrix[0][1] + matrix[1][1] + matrix[2][1]) * sqrt_magnitude,(matrix[0][2] + matrix[1][2] + matrix[2][2]) * sqrt_magnitude };
			float vector_magnitude = pow(direction_vector[0], 2) + pow(direction_vector[1], 2) + pow(direction_vector[2], 2);
			
			//TODO: get intersection point P_int of line formed by vector and direction on screen_plane_adjusted
			
			
			//TODO: send data to serial
			
			

		}

		


		
		Sleep(500);//expected polling rate of 200Hz
		if (check_end_signal()) {//if end signal received, end thread
			std::cout << "thread for device "<< device_number << " ended" << endl;
			return;
		}
	}


}


float *get_coordinate_trigger_press(int device_number, float *recv_direction_vector) { //get controller coordinates on trigger press 
	float *matrix[3];
	float *ret_value = new float[3];
	bool ret_defined = false;
	for (;;) {
		bool tracking_values_available = false;
		tracking_device_mutex.lock();//thread safety for multiple controllers
		TrackedDevicePose_t tracked_device_pose;
		VRControllerState_t controllerState;
		

		vr_context->GetControllerStateWithPose(//get controller state and translation
			TrackingUniverseStanding, device_number, &controllerState,
			sizeof(controllerState), &tracked_device_pose);
		tracking_device_mutex.unlock();

		if ((tracked_device_pose.bPoseIsValid && tracked_device_pose.bDeviceIsConnected))
		{
			matrix[0] = tracked_device_pose.mDeviceToAbsoluteTracking.m[0];
			matrix[1] = tracked_device_pose.mDeviceToAbsoluteTracking.m[1];
			matrix[2] = tracked_device_pose.mDeviceToAbsoluteTracking.m[2];
			tracking_values_available = true;
		}
		if (tracking_values_available) {//then process controller position

			for (int j = 0; j < vr::k_unControllerStateAxisCount; j++)//check for trigger press
			{
				vr::ETrackedDeviceProperty prop = (vr::ETrackedDeviceProperty)(vr::Prop_Axis0Type_Int32 + j);
				vr::EVRControllerAxisType type = (vr::EVRControllerAxisType)vr::VRSystem()->GetInt32TrackedDeviceProperty(device_number, prop);
				if (type == vr::k_eControllerAxis_Trigger)
				{
					if (j==1 && controllerState.rAxis[j].x > 0.5) {
						ret_value[0] = matrix[0][3];
						ret_value[1] = matrix[1][3];
						ret_value[2] = matrix[2][3];
						float sqrt_magnitude = sqrt(1 / 3.00);
						float direction_vector[3] = { (matrix[0][0] + matrix[1][0] + matrix[2][0]) * sqrt_magnitude, (matrix[0][1] + matrix[1][1] + matrix[2][1]) * sqrt_magnitude,(matrix[0][2] + matrix[1][2] + matrix[2][2]) * sqrt_magnitude };
						

						if (!ret_defined) {
							cout << "Now please release the trigger" << j << endl;
						}
						ret_defined = true;
						
						recv_direction_vector = direction_vector;
						vr_context->TriggerHapticPulse(device_number, 0, 300);
						
					}
					else if(j == 1 &&  ret_defined && controllerState.rAxis[j].x<0.5){

						return ret_value;
					}
				}
				

			}
		}

	}
}





float dot_product(float v1[3], float v2[3]) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void multiplyMatrices(float **firstMatrix, float **secondMatrix, float **mult)
{
	int i, j, k;
	// Initializing elements of matrix mult to 0.
	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{
			mult[i][j] = 0;
		}
	}

	// Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
	for (i = 0; i < 3; ++i)
	{
		for (j = 0; j < 3; ++j)
		{

			for (k = 0; k < 3; ++k)
			{
				mult[i][j] += (firstMatrix[i][k] * secondMatrix[k][j]);
			}
		}
	}
}



void vector_scalar_mult(float **x, float scalar) {
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			x[i][j] = x[i][j] * scalar;
		}
}

void sum_matrix(float **x, float **y) {//x will be updated
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
		{
			x[i][j] = x[i][j] + y[i][j];
		}
}


float **get_rotation_matrix(float f[3], float t[3]) {// rotation of a into b

	float mag_f = pow((pow(f[0], 2) + pow(f[1], 2) + pow(f[2], 2)), 0.5);
	float mag_t = pow((pow(t[0], 2) + pow(t[1], 2) + pow(t[2], 2)), 0.5);

	//std::cout << "mag_f=" << mag_f << endl;
	//std::cout << "mag_t=" << mag_t << endl;

	f[0] = f[0] / mag_f;
	f[1] = f[1] / mag_f;
	f[2] = f[2] / mag_f;
	t[0] = t[0] / mag_t;
	t[1] = t[1] / mag_t;
	t[2] = t[2] / mag_t;


	//std::cout << "new vector f=" << f[0] << "," << f[1] << "," << f[2] << endl;
	//std::cout << "new vector t=" << t[0] << "," << t[1] << "," << t[2] << endl;

	float v[3];
	float s;


	v[0] = (f[1] * t[2]) - (f[2] * t[1]);
	v[1] = -((f[0] * t[2]) - (f[2] * t[0]));
	v[2] = (f[0] * t[1]) - (f[1] * t[0]);

	//std::cout << "v=aXb=" << v[0] << "," << v[1] << "," << v[2] << endl;


	s = pow((pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2)), 0.5);
	//std::cout << "sin value=" << s << endl;



	float c = dot_product(f, t);
	//std::cout << "cos value=" << c << endl;


	float** u_hat;
	float** u_hat_sq;
	float** identity;
	u_hat = new float*[3];
	u_hat_sq = new float*[3];
	identity = new float*[3];
	for (int i = 0; i < 3; i++) {
		u_hat[i] = new float[3];
		u_hat_sq[i] = new float[3];
		identity[i] = new float[3];
		for (int j = 0; j < 3; j++) {
			u_hat[i][j] = 0;
			if (i == j) {
				identity[i][j] = 1;
			}
			else {
				identity[i][j] = 0;
			}
		}
	}
	//std::cout << "Identity Matrix" << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << identity[i][j] << " ";
		}
		std::cout << endl;
	}



	u_hat[0][1] = -v[2];
	u_hat[1][0] = v[2];
	u_hat[0][2] = v[1];
	u_hat[2][0] = -v[1];
	u_hat[1][2] = -v[0];
	u_hat[2][1] = v[0];




	multiplyMatrices(u_hat, u_hat, u_hat_sq);
	//std::cout << "u_hat Matrix" << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << u_hat[i][j] << " ";
		}
		std::cout << endl;
	}
	//std::cout << "u_hat square" << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			std::cout << u_hat_sq[i][j] << " ";
		}
		std::cout << endl;
	}



	vector_scalar_mult(u_hat_sq, (1 / (1 + c)));



	sum_matrix(u_hat_sq, u_hat);
	sum_matrix(u_hat_sq, identity);


	return u_hat_sq;



}
