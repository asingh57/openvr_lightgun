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
#include "wtypes.h"


#define ARDUINO_WAIT_TIME 2000



class SerialPort
{
private:
	HANDLE handler;
	bool connected;
	COMSTAT status;
	DWORD errors;
public:
	SerialPort(char *portName);
	~SerialPort();

	int readSerialPort(char *buffer, unsigned int buf_size);
	bool writeSerialPort(int n);
	bool isConnected();
};

SerialPort::SerialPort(char *portName)
{
	this->connected = false;

	this->handler = CreateFileA(static_cast<LPCSTR>(portName),
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);
	if (this->handler == INVALID_HANDLE_VALUE) {
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			printf("ERROR: Handle was not attached. Reason: %s not available\n", portName);
		}
		else
		{
			printf("ERROR!!!");
		}
	}
	else {
		DCB dcbSerialParameters = { 0 };

		if (!GetCommState(this->handler, &dcbSerialParameters)) {
			printf("failed to get current serial parameters");
		}
		else {
			dcbSerialParameters.BaudRate = CBR_9600;
			dcbSerialParameters.ByteSize = 8;
			dcbSerialParameters.StopBits = ONESTOPBIT;
			dcbSerialParameters.Parity = NOPARITY;
			dcbSerialParameters.fDtrControl = DTR_CONTROL_ENABLE;

			if (!SetCommState(handler, &dcbSerialParameters))
			{
				printf("ALERT: could not set Serial port parameters\n");
			}
			else {
				this->connected = true;
				PurgeComm(this->handler, PURGE_RXCLEAR | PURGE_TXCLEAR);
				Sleep(ARDUINO_WAIT_TIME);
			}
		}
	}
}

SerialPort::~SerialPort()
{
	if (this->connected) {
		this->connected = false;
		CloseHandle(this->handler);
	}
}

int SerialPort::readSerialPort(char *buffer, unsigned int buf_size)
{
	DWORD bytesRead;
	unsigned int toRead;

	ClearCommError(this->handler, &this->errors, &this->status);

	if (this->status.cbInQue > 0) {
		if (this->status.cbInQue > buf_size) {
			toRead = buf_size;
		}
		else toRead = this->status.cbInQue;
	}

	if (ReadFile(this->handler, buffer, toRead, &bytesRead, NULL)) return bytesRead;

	return 0;
}

bool SerialPort::writeSerialPort(int number)
{
	DWORD bytesSend;

	std::string s = std::to_string(number);
	char const *pchar = s.c_str();


	if (!WriteFile(this->handler, pchar, s.length(), &bytesSend, 0)) {
		ClearCommError(this->handler, &this->errors, &this->status);
		return false;
	}
	else return true;
}

bool SerialPort::isConnected()
{
	return this->connected;
}



using namespace std;
using namespace vr;

set<int> available_controllers;


void GetDesktopResolution(int& horizontal, int& vertical)
{
	RECT desktop;
	// Get a handle to the desktop window
	const HWND hDesktop = GetDesktopWindow();
	// Get the size of screen to the variable desktop
	GetWindowRect(hDesktop, &desktop);
	// The top left corner will have coordinates (0,0)
	// and the bottom right corner will have coordinates
	// (horizontal, vertical)
	horizontal = desktop.right;
	vertical = desktop.bottom;
}


std::mutex end_signal_mutex;
bool end_signal=false;
std::mutex tracking_device_mutex;
float screen_plane_input[2][2][3];

float screen_plane_adjusted[4]; //form ax,by,cz,d
float *screen_plane_corners[4];//four corners of the screen plane correspond to the screen
float screen_normal_unit_vector[3];

float screen_in_2d[4][3];
float loc0[3], locx[3], locy[3], normal[3];


int hor_resolution;
int vert_resolution;


void send_end_signal();
bool check_end_signal();
void run_mouse_emulation(int device_number, string com_port);

string ftos(float f, int precision);	// float to string with 2-decimal precision
string vftos(float v[3], int precision);	// float vector to string with 2-decimal precisions
string GetTrackedDeviceString(vr::IVRSystem*, vr::TrackedDeviceIndex_t, vr::TrackedDeviceProperty, vr::TrackedPropertyError *peError = NULL);
string GetTrackedDeviceClassString(vr::ETrackedDeviceClass td_class);



float dot_product2(float v1[3], float v2[3]);
void multiplyMatrices(float *firstMatrix[3], float *secondMatrix[3], float *mult[3]);
void matrix_scalar_mult(float *x[3], float scalar);
void sum_matrix(float **x, float **y);//x will be updated
float **get_rotation_matrix(float f[3], float t[3]);// rotation of a into b


SerialPort *arduino[64];




vr::IVRSystem* vr_context;

bool app_end = false;
string driver_name, driver_serial;
string tracked_device_type[vr::k_unMaxTrackedDeviceCount];

int init_OpenVR();
void exit();
float *get_coordinate_trigger_press(int device_number, float* ret_val ,float recv_direction_vector[3]=new float[3]);



void Quad_to_Logical_Cell(float x,float y,int *L_Out){
    
  
/*
  'WJW 7-13-15
  'This function performs a coordinate transform from X,Y space to the normalized L,M.
  '
  'If a point {is within {0,1} on both axes, it is within the transformed unit square.
  'Qx,Qy vectors contain the 4 coordinates of the corners - x and y values, respectively, ordered as indicated below:
  '
  'The unit cell L(l,m) corresponding to Q(x,y) is oriented as:
  'L0(x=0,y=0),L1(0,1), L2(1,1), L3(1,0).  The order matters.
  'The following represent an algebraic solution to the system:
  'l=a1 + b1x + c1y + d1xy
  'm=a2 + b2x + c2y + d2xy
*/


    float Qx[4]={screen_in_2d[0][0],screen_in_2d[1][0],screen_in_2d[2][0],screen_in_2d[3][0]};
    float Qy[4]={screen_in_2d[0][1],screen_in_2d[1][1],screen_in_2d[2][1],screen_in_2d[3][1]};


    float ax = (x - Qx[0]) + (Qx[1] - Qx[0]) * (y - Qy[0]) / (Qy[0] - Qy[1]);
    float a3x = (Qx[3] - Qx[0]) + (Qx[1] - Qx[0]) * (Qy[3] - Qy[0]) / (Qy[0] - Qy[1]);
    float a2x = (Qx[2] - Qx[0]) + (Qx[1] - Qx[0]) * (Qy[2] - Qy[0]) / (Qy[0] - Qy[1]);
    float ay = (y - Qy[0]) + (Qy[3] - Qy[0]) * (x - Qx[0]) / (Qx[0] - Qx[3]);
    float a1y = (Qy[1] - Qy[0]) + (Qy[3] - Qy[0]) * (Qx[1] - Qx[0]) / (Qx[0] - Qx[3]);
    float a2y = (Qy[2] - Qy[0]) + (Qy[3] - Qy[0]) * (Qx[2] - Qx[0]) / (Qx[0] - Qx[3]);
    float bx = x * y - Qx[0] * Qy[0] + (Qx[1] * Qy[1] - Qx[0] * Qy[0]) * (y - Qy[0]) / (Qy[0] - Qy[1]);
    float b3x = Qx[3] * Qy[3] - Qx[0] * Qy[0] + (Qx[1] * Qy[1] - Qx[0] * Qy[0]) * (Qy[3] - Qy[0]) / (Qy[0] - Qy[1]);
    float b2x = Qx[2] * Qy[2] - Qx[0] * Qy[0] + (Qx[1] * Qy[1] - Qx[0] * Qy[0]) * (Qy[2] - Qy[0]) / (Qy[0] - Qy[1]);
    float by = x * y - Qx[0] * Qy[0] + (Qx[3] * Qy[3] - Qx[0] * Qy[0]) * (x - Qx[0]) / (Qx[0] - Qx[3]);
    float b1y = Qx[1] * Qy[1] - Qx[0] * Qy[0] + (Qx[3] * Qy[3] - Qx[0] * Qy[0]) * (Qx[1] - Qx[0]) / (Qx[0] - Qx[3]);
    float b2y = Qx[2] * Qy[2] - Qx[0] * Qy[0] + (Qx[3] * Qy[3] - Qx[0] * Qy[0]) * (Qx[2] - Qx[0]) / (Qx[0] - Qx[3]);

    L_Out[1] = vert_resolution * (1-((ax / a3x) + (1 - a2x / a3x) * (bx - b3x * ax / a3x) / (b2x - b3x * a2x / a3x)));
    L_Out[0] = hor_resolution * (((ay / a1y) + (1 - a2y / a1y) * (by - b1y * ay / a1y) / (b2y - b1y * a2y / a1y)));

}


void cross_product(float *a, float *b, float *ret_value) {
	int x = 0;
	int y = 1;
	int z = 2;


	ret_value[0] = a[y] * b[z] - a[z] * b[y];
	ret_value[1] = a[z] * b[x] - a[x] * b[z];
	ret_value[2] = a[x] * b[y] - a[y] * b[x];


	return;
}
float dot_product(float *a, float *b) {
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float magnitude(float *vectorx) {
	return pow((pow(vectorx[0], 2) + pow(vectorx[1], 2) + pow(vectorx[2], 2)), 0.5);
}
void subtract_vectors(float *a, float *b, float *ret_value) {

	ret_value[0] = a[0] - b[0];
	ret_value[1] = a[1] - b[1];
	ret_value[2] = a[2] - b[2];
}

void add_vectors(float *a, float *b, float *ret_value) {

	ret_value[0] = a[0] + b[0];
	ret_value[1] = a[1] + b[1];
	ret_value[2] = a[2] + b[2];



	return;
}

void convert_to_2d(float *p, float *ret_value) {

	float r1[3];
	float r2[3];
	subtract_vectors(p, loc0, r1);
	subtract_vectors(p, loc0, r2);
	ret_value[0] = dot_product(r1, locx);
	ret_value[1] = dot_product(r2, locy);
	ret_value[2] = 0;
	return;
}


void unit_normal_vector(float *v1, float *v2, float *ret_val) {

	float dir[3] = { v1[0] - v2[0],v1[1] - v2[1],0 };
	float mag = magnitude(dir);

	ret_val[0] = dir[0] / mag;
	ret_val[1] = dir[1] / mag;
	ret_val[2] = 0;
	return;

}

void get_ptr_pos(float *pt_of_screen_projection, int* ret_val) {
	//using https://math.stackexchange.com/questions/13404/mapping-irregular-quadrilateral-to-a-rectangle
	int x = 0;
	int y = 1;

	float p[3], *p0, *p1, *p2, *p3, n0[3], n1[3], n2[3], n3[3];
	float u, v;
	p0 = screen_in_2d[0];
	p1 = screen_in_2d[1];
	p2 = screen_in_2d[3];
	p3 = screen_in_2d[2];

	
	unit_normal_vector(p0, p3, n0);
	unit_normal_vector(p0, p1, n1);
	unit_normal_vector(p1, p2, n2);
	unit_normal_vector(p2, p3, n3);

	
	float vs1[3];
	float vs2[3];
	float vs3[3];
	float vs4[3];

	//	following part is in the loop
	convert_to_2d(pt_of_screen_projection, p);

	subtract_vectors(p, p0, vs1);
	subtract_vectors(p, p2, vs2);
	subtract_vectors(p, p0, vs3);
	subtract_vectors(p, p3, vs4);

	u = dot_product(vs1, n0);
	u = u / (u + dot_product(vs2, n2));
	
	v = dot_product(vs3, n1);
	v = v / (v + dot_product(vs4, n3));
	

	float a = n0[x], b = n0[y], c = -dot_product(p0, n0);
	float d = n0[x] + n2[x], e = n0[y] + n2[y], f = -dot_product(p0, n0) - dot_product(p2, n2);
	float g = n1[x], h = n1[y], i = -dot_product(p0, n1);
	float j = n1[x] + n3[x], k = n1[y] + n3[y], l = -dot_product(p0, n1) - dot_product(p2, n3);

	
	float uDA = u * (d - a);
	float uEB = u * (e - b);
	float uFC = u * (f - c);
	float vJG = u * (j - g);
	float vKH = u * (k - h);
	float vLI = u * (l - i);

	cout << (vKH*uFC - vLI * uEB) << endl;
	cout << (vJG*uEB - vKH * uDA) << endl;
	cout << (vLI*uDA - uFC * vJG) << endl;
	cout << (vJG*uEB - vKH * uDA) << endl;
	cout << (vKH*uFC - vLI * uEB) / (vJG*uEB - vKH * uDA) << endl;
	cout << (vLI*uDA - uFC * vJG) / (vJG*uEB - vKH * uDA) << endl;


	ret_val[0] = hor_resolution * (vKH*uFC - vLI * uEB) / (vJG*uEB - vKH * uDA);
	ret_val[1] = vert_resolution * (vLI*uDA - uFC * vJG) / (vJG*uEB - vKH * uDA);

}


void initialise_2d() {
	//use screen_plane_corners
	float normal[3];

	loc0[0] = screen_plane_corners[0][0];
	loc0[1] = screen_plane_corners[0][1];
	loc0[2] = screen_plane_corners[0][2];

	locx[0] = screen_plane_corners[1][0];
	locx[1] = screen_plane_corners[1][1];
	locx[2] = screen_plane_corners[1][2];

	float r1[3];
	float r2[3];
	float r3[3];
	float r4[2];
	float r6[2];
	float r7[2];

	subtract_vectors(screen_plane_corners[2], loc0, r1);
	cross_product(locx, r1, normal);
	cross_product(normal, locx, locy);


	float mag_locx = magnitude(locx);
	float mag_locy = magnitude(locy);

	locx[0] = locx[0] / mag_locx;
	locx[1] = locx[1] / mag_locx;
	locx[2] = locx[2] / mag_locx;
	locy[0] = locy[0] / mag_locy;
	locy[1] = locy[1] / mag_locy;
	locy[2] = locy[2] / mag_locy;



	convert_to_2d(screen_plane_corners[0], screen_in_2d[0]);
	convert_to_2d(screen_plane_corners[1], screen_in_2d[1]);
	convert_to_2d(screen_plane_corners[2], screen_in_2d[2]);
	convert_to_2d(screen_plane_corners[3], screen_in_2d[3]);

}

void scalar_mult_vector(float scalar, float *vectorx, float *ret_value) {
	ret_value[0] = vectorx[0] * scalar;
	ret_value[1] = vectorx[1] * scalar;
	ret_value[2] = vectorx[2] * scalar;
	return;
}

void back_to_3d(float *vectorx, float *ret_value) {
	float r1[3];
	float r2[3];
	float r4[3];

	scalar_mult_vector(vectorx[0], locx, r2);
	scalar_mult_vector(vectorx[1], locy, r1);
	add_vectors(loc0, r2, r4);
	add_vectors(r4, r1, ret_value);

	return;
}



int main()
{
	

	std::cout << "OpenVR_LightGun!\n" << endl;
	std::cout << (("Driver name: " + driver_name).c_str()) << " ";

	std::cout << (("Driver serial ID: " + driver_serial).c_str()) << endl;
	if (init_OpenVR() != 0) { std::cout << "Failed to initialise OpenVR"; return -1; }
	int emulator_count = 0;
	thread emulator_threads[64];

	bool screen_coordinates_configured = false;
	
	int device_number_arr[64];
	string com_port_arr[64];
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

					std::cout << "Please bring the controller to the bottom left edge of the screen and press the trigger" << endl;
					get_coordinate_trigger_press(device_number, screen_plane_input[0][0]);
					std::cout << "Please bring the controller to the bottom right edge of the screen and press the trigger" << endl;
					get_coordinate_trigger_press(device_number, screen_plane_input[0][1]);
					std::cout << "Please bring the controller to the top right edge of the screen and press the trigger" << endl;
					get_coordinate_trigger_press(device_number, screen_plane_input[1][1]);
					std::cout << "Please bring the controller to the top left edge of the screen and press the trigger" << endl;
					get_coordinate_trigger_press(device_number, screen_plane_input[1][0]);

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


				screen_plane_corners[0] = screen_plane_input[0][0];
				screen_plane_corners[1] = screen_plane_input[0][1];
				screen_plane_corners[2] = screen_plane_input[1][1];

				float a = screen_plane_adjusted[0], b = screen_plane_adjusted[1], c = screen_plane_adjusted[2], d = screen_plane_adjusted[3];
				float p = screen_plane_input[1][0][0], q = screen_plane_input[1][0][1], r = screen_plane_input[1][0][2];
				float t = (-d - a * p - b * q - c * r) / (a*a + b * b + c * c);
				screen_plane_corners[3] = new float[3];
				screen_plane_corners[3][0]= p + a * t;
				screen_plane_corners[3][1] = q + b * t;
				screen_plane_corners[3][2] = r + c * t; 

				initialise_2d();// maps the screen plane onto z=0 plane
				


			}

			std::cout << "Enter a valid com port number for this light_gun(you can find this in windows device manager)" << endl;
			string com_port;
			cin >> com_port;

			const string port_str = "\\\\.\\COM" + com_port;
			char char_array[32];

			strcpy_s(char_array, port_str.c_str());
			arduino[device_number] = new SerialPort(char_array);
			std::cout << "Make sure your arduino leonardo with arduino_program.ino on it is connected \n" << endl;
			std::cout << "Arduino Calibration stage. Please make sure you are not moving the mouse until allowed to do so\n" << endl;
			POINT ptOld;
			POINT ptNew;
			std::cout << "Press any key to start" << endl;
			char rr;
			cin >> rr;
			GetCursorPos(&ptOld);
			//send char signal to serial
			arduino[device_number]->writeSerialPort(1);
			Sleep(500);
			GetCursorPos(&ptNew);
			arduino[device_number]->writeSerialPort(ptNew.x - ptOld.x);
			//send it to serial ptNew.x - ptOld.x
			std::cout << "Calibration complete. You can move your mouse now" << endl;


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
	float abs_position_controller_cursor[3];
	get_coordinate_trigger_press(device_number, abs_position_controller_cursor, direction_vector_cursor_controller);//x,y,z coordinates
	
	//find rotation matrix between direction vector and screen normal vector
	float **rotation_matrix = get_rotation_matrix(direction_vector_cursor_controller, screen_normal_unit_vector);

	float adjusted_direction_vector[3];
	float a = screen_plane_adjusted[0], b = screen_plane_adjusted[1], c = screen_plane_adjusted[2], d = screen_plane_adjusted[3];
	float pt_of_screen_projection[3];

	GetDesktopResolution(hor_resolution,vert_resolution);

	GetCursorPos(&cursor_position);

	//TODO: find direction vector between screen_normal_unit_vector and direction_vector_cursor_controller

	int axis_arr[2];


	float proj_in_2d[2];
	VREvent_t event;

	for (;;) {
		//button tracking only works when controller plugged in

		tracking_values_available = false;
		tracking_device_mutex.lock();//thread safety for multiple controllers
		TrackedDevicePose_t tracked_device_pose;
		VRControllerState_t controllerState;
		float sqrt_magnitude = sqrt(1 / 3.00);

		vr_context->GetControllerStateWithPose(//get controller state and translation
			TrackingUniverseStanding, device_number, &controllerState,
			sizeof(controllerState), &tracked_device_pose);
		tracking_device_mutex.unlock();

		int deltax; int deltay;

		float a = screen_plane_adjusted[0], b = screen_plane_adjusted[1], c = screen_plane_adjusted[2], d = screen_plane_adjusted[3];
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
			float direction_vector[3] = { (matrix[0][0] + matrix[0][1] + matrix[0][2]) * sqrt_magnitude, (matrix[1][0] + matrix[1][1] + matrix[1][2]) * sqrt_magnitude,(matrix[2][0] + matrix[2][0] + matrix[2][2]) * sqrt_magnitude };
			//float vector_magnitude = pow(direction_vector[0], 2) + pow(direction_vector[1], 2) + pow(direction_vector[2], 2);
			
			cout << "abs_posn"<< abs_position[0] << "," << abs_position[1] << "," << abs_position[2] << endl;
			cout << "dir_vector" << direction_vector[0] << "," << direction_vector[1] << "," << direction_vector[2] << endl;

			
			//adjusted_direction_vector[0] = rotation_matrix[0][0]*direction_vector[0]+ rotation_matrix[0][1] * direction_vector[1]+ rotation_matrix[0][2] * direction_vector[2];
			//adjusted_direction_vector[1] = rotation_matrix[1][0] * direction_vector[0] + rotation_matrix[1][1] * direction_vector[1] + rotation_matrix[1][2] * direction_vector[2];
			//adjusted_direction_vector[2] = rotation_matrix[2][0] * direction_vector[0] + rotation_matrix[2][1] * direction_vector[1] + rotation_matrix[2][2] * direction_vector[2];

			adjusted_direction_vector[0] = direction_vector[0];
			adjusted_direction_vector[1] = direction_vector[1];
			adjusted_direction_vector[2] = direction_vector[2];


			float p = abs_position[0], q = abs_position[1], r = abs_position[2];
			float l = adjusted_direction_vector[0], m = adjusted_direction_vector[1], n = adjusted_direction_vector[2];
			float t = (-d - a * p - b * q - c * r) / (a*l + b * m + c * n);
			pt_of_screen_projection[0] = p + l * t;
			pt_of_screen_projection[1] = q + m * t;
			pt_of_screen_projection[2] = r + n * t;
			cout << "pt_of_screen_projection" << pt_of_screen_projection[0] << "," << pt_of_screen_projection[1] << "," << pt_of_screen_projection[2] << endl;


			//map point onto the display resolution

			//assuming top and bottom of the screen are parallel and left and right have equal lengths
			//get ratio of distance of pt from top and bottom
			//map that onto y coordinate


			convert_to_2d(pt_of_screen_projection, proj_in_2d);

			Quad_to_Logical_Cell(proj_in_2d[0], proj_in_2d[1],axis_arr);
			//get_ptr_pos(pt_of_screen_projection, axis_arr);

			cout << "Axis arr" << axis_arr[0] <<"," << axis_arr[1] <<endl;

			if (axis_arr[0]<=hor_resolution&& axis_arr[1] <= vert_resolution && axis_arr[0]>=0 && axis_arr[1] >= 0) {
				
				deltax = axis_arr[0] - cursor_position.x;
				deltay = axis_arr[1] - cursor_position.y;
				if (abs(deltax) > 2000 || abs(deltay)>2000) {
					GetCursorPos(&cursor_position);
				}
				else {
					arduino[device_number]->writeSerialPort(deltax);
					arduino[device_number]->writeSerialPort(deltay);
					Sleep(5);//expected polling rate of 200Hz
					GetCursorPos(&cursor_position);
				
				
				}

				
			}


			//TODO: send data to serial

			//now map this onto
			

		}


		
		
		

		if (check_end_signal()) {//if end signal received, end thread
			std::cout << "thread for device "<< device_number << " ended" << endl;
			return;
		}
	}


}


float *get_coordinate_trigger_press(int device_number, float *ret_value,float *direction_vector) { //get controller coordinates on trigger press 
	float *matrix[3];
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
						direction_vector[0] = (matrix[0][0] + matrix[1][0] + matrix[2][0]) * sqrt_magnitude;
						direction_vector[1] = (matrix[0][1] + matrix[1][1] + matrix[2][1]) * sqrt_magnitude;
						direction_vector[2] = (matrix[0][2] + matrix[1][2] + matrix[2][2]) * sqrt_magnitude;
						if (!ret_defined) {
							cout << "Now please release the trigger" << j << endl;
						}
						ret_defined = true;
						
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





float dot_product2(float v1[3], float v2[3]) {
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



void matrix_scalar_mult(float **x, float scalar) {
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



	float c = dot_product2(f, t);
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
	


	u_hat[0][1] = -v[2];
	u_hat[1][0] = v[2];
	u_hat[0][2] = v[1];
	u_hat[2][0] = -v[1];
	u_hat[1][2] = -v[0];
	u_hat[2][1] = v[0];




	multiplyMatrices(u_hat, u_hat, u_hat_sq);
	//std::cout << "u_hat Matrix" << endl;
	

	matrix_scalar_mult(u_hat_sq, (1 / (1 + c)));



	sum_matrix(u_hat_sq, u_hat);
	sum_matrix(u_hat_sq, identity);


	return u_hat_sq;



}
