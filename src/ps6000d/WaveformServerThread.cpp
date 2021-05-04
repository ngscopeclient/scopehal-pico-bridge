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
float InterpolateTriggerTime(int16_t* buf);

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

	size_t numSamples = 0;
	uint16_t numchans;
	while(!g_waveformThreadQuit)
	{
		int16_t ready;
		{
			lock_guard<mutex> lock(g_mutex);
			ps6000aIsReady(g_hScope, &ready);
		}

		if( (ready == 0) || (!g_triggerArmed) )
		{
			usleep(1000);
			continue;
		}

		{
			lock_guard<mutex> lock(g_mutex);

			//Stop the trigger
			auto status = ps6000aStop(g_hScope);
			if(PICO_OK != status)
				LogFatal("ps6000aStop failed (code 0x%x)\n", status);

			//Verify it's actually stopped

			//Set up buffers if needed
			if(g_memDepthChanged || waveformBuffers.empty())
			{
				LogDebug("Reallocating buffers\n");

				for(size_t i=0; i<g_numChannels; i++)
				{
					ps6000aSetDataBuffer(g_hScope, (PICO_CHANNEL)i, NULL,
						0, PICO_INT16_T, 0, PICO_RATIO_MODE_RAW, PICO_CLEAR_ALL);
				}
				for(size_t i=0; i<g_numChannels; i++)
				{
					//Allocate memory if needed
					if(waveformBuffers[i])
						delete[] waveformBuffers[i];
					waveformBuffers[i] = new int16_t[g_captureMemDepth];
					memset(waveformBuffers[i], 0x00, g_captureMemDepth * sizeof(int16_t));

					//Give it to the scope, removing any other buffer we might have
					status = ps6000aSetDataBuffer(g_hScope, (PICO_CHANNEL)i, waveformBuffers[i],
						g_captureMemDepth, PICO_INT16_T, 0, PICO_RATIO_MODE_RAW, PICO_ADD);
					if(status != PICO_OK)
						LogFatal("ps6000aSetDataBuffer failed (code 0x%x)\n", status);
				}

				g_memDepthChanged = false;
			}

			//Download the data from the scope
			numSamples = g_captureMemDepth;
			int16_t overflow = 0;
			status = ps6000aGetValues(g_hScope, 0, &numSamples, 1, PICO_RATIO_MODE_RAW, 0, &overflow);
			if(PICO_OK != status)
				LogFatal("ps6000aGetValues (code 0x%x)\n", status);

			//Figure out how many channels are active in this capture
			numchans = 0;
			for(size_t i=0; i<g_numChannels; i++)
			{
				if(g_channelOnDuringArm[i])
					numchans ++;
			}
		}

		//Send the channel count to the client
		client.SendLooped((uint8_t*)&numchans, sizeof(numchans));

		//Send sample rate to the client
		client.SendLooped((uint8_t*)&g_sampleIntervalDuringArm, sizeof(g_sampleIntervalDuringArm));

		//TODO: send overflow flags to client

		//Interpolate trigger position if we're using a simple level trigger
		float trigphase = InterpolateTriggerTime(waveformBuffers[g_triggerChannel]);

		//Send data for each channel to the client
		for(size_t i=0; i<g_numChannels; i++)
		{
			if(g_channelOnDuringArm[i])
			{
				//Send channel ID, scale, offset, and memory depth
				client.SendLooped((uint8_t*)&i, sizeof(i));
				client.SendLooped((uint8_t*)&numSamples, sizeof(numSamples));
				float scale = g_roundedRange[i] / 32512;
				float offset = g_offsetDuringArm[i];
				float config[3] = {scale, offset, trigphase};
				client.SendLooped((uint8_t*)&config, sizeof(config));

				//Send the actual waveform data
				client.SendLooped((uint8_t*)waveformBuffers[i], g_captureMemDepth * sizeof(int16_t));
			}
		}

		//Re-arm the trigger if doing repeating triggers
		if(g_triggerOneShot)
			g_triggerArmed = false;
		else
		{
			lock_guard<mutex> lock(g_mutex);

			if(g_captureMemDepth != g_memDepth)
				g_memDepthChanged = true;

			//Restart
			StartCapture(false);
		}
	}

	//Clean up temporary buffers
	for(auto it : waveformBuffers)
		delete[] it.second;
}

float InterpolateTriggerTime(int16_t* buf)
{
	float trigscale = g_roundedRange[g_triggerChannel] / 32512;
	float trigoff = g_offsetDuringArm[g_triggerChannel];

	float fa = buf[g_triggerSampleIndex-1] * trigscale + trigoff;
	float fb = buf[g_triggerSampleIndex] * trigscale + trigoff;

	//no need to divide by time, sample spacing is normalized to 1 timebase unit
	float slope = (fb - fa);
	float delta = g_triggerVoltage - fa;
	return delta / slope;
}
