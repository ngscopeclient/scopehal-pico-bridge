/***********************************************************************************************************************
*                                                                                                                      *
* ps6000d                                                                                                              *
*                                                                                                                      *
* Copyright (c) 2012-2021 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author Andrew D. Zonenberg
	@brief SCPI server. Control plane traffic only, no waveform data.
 */
#include "ps6000d.h"
#include <string.h>

using namespace std;

volatile bool g_waveformThreadQuit = false;

//TODO: mutexing (driver is NOT thread safe!)

void WaveformServerThread()
{
	Socket client = g_dataSocket.Accept();
	LogVerbose("Client connected to data plane socket\n");

	if(!client.IsValid())
		return;
	if(!client.DisableNagle())
		LogWarning("Failed to disable Nagle on socket, performance may be poor\n");

	//Set up buffers
	map<size_t, int16_t*> waveformBuffers;
	for(size_t i=0; i<g_numChannels; i++)
	{
		//Allocate memory if needed
		waveformBuffers[i] = new int16_t[g_memDepth];
		memset(waveformBuffers[i], 0x00, g_memDepth * sizeof(int16_t));

		//Give it to the scope
		auto status = ps6000aSetDataBuffer(g_hScope, (PICO_CHANNEL)i, waveformBuffers[i],
			g_memDepth, PICO_INT16_T, 0, PICO_RATIO_MODE_RAW, PICO_ADD);
		if(status != PICO_OK)
		{
			LogFatal("ps6000aSetDataBuffer failed (code %d)\n", status);
		}
	}

	while(!g_waveformThreadQuit)
	{
		int16_t status;
		ps6000aIsReady(g_hScope, &status);

		if( (status == 0) || (!g_triggerArmed) )
		{
			usleep(1000);
			continue;
		}

		//Stop the trigger
		ps6000aStop(g_hScope);

		//Download the data from the scope
		size_t numSamples = g_captureMemDepth;
		int16_t overflow = 0;
		status = ps6000aGetValues(g_hScope, 0, &numSamples, 1, PICO_RATIO_MODE_RAW, 0, &overflow);
		if(PICO_OK != status)
			LogFatal("ps6000aGetValues (code %d)\n", status);

		//Figure out how many channels are active in this capture
		uint16_t numchans = 0;
		for(size_t i=0; i<g_numChannels; i++)
		{
			if(g_channelOnDuringArm[i])
				numchans ++;
		}

		//Send the channel count to the client
		client.SendLooped((uint8_t*)&numchans, sizeof(numchans));

		//TODO: send overflow flags to client

		//Send data for each channel to the client
		for(size_t i=0; i<g_numChannels; i++)
		{
			if(g_channelOnDuringArm[i])
			{
				//Send channel ID, scale, and memory depth
				client.SendLooped((uint8_t*)&i, sizeof(i));
				client.SendLooped((uint8_t*)&numSamples, sizeof(numSamples));
				float scale = g_roundedRange[i] / 32512;
				client.SendLooped((uint8_t*)&scale, sizeof(scale));

				//Send the actual waveform data
				client.SendLooped((uint8_t*)waveformBuffers[i], g_captureMemDepth * sizeof(uint16_t));
			}
		}

		//Restart
		status = ps6000aRunBlock(g_hScope, g_captureMemDepth/2, g_captureMemDepth/2, 2, NULL, 0, NULL, NULL);
		if(status != PICO_OK)
			LogFatal("ps6000aRunBlock failed, code %d\n", status);
	}
}
