// OpenVR_LightGun.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "C:/Users/abhi_/Documents/OpenVR_LightGun/openvr/headers/openvr.h"
#pragma comment(lib, "C:/Users/abhi_/Documents/OpenVR_LightGun/openvr/lib/win32/openvr_api.lib")

using namespace vr;

IVRSystem* vr_pointer = NULL;

int main()
{
	EVRInitError eError = VRInitError_None;
	vr_pointer = VR_Init(&eError, VRApplication_Background);
	if (eError != VRInitError_None)
	{
		vr_pointer = NULL;
		printf("Unable to init VR runtime: %s \n",
			VR_GetVRInitErrorAsEnglishDescription(eError));
		//exit(EXIT_FAILURE);
	}

    std::cout << "Hello World!\n"; 
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
