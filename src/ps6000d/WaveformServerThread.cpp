/***********************************************************************************************************************
*                                                                                                                      *
* ps6000d                                                                                                              *
*                                                                                                                      *
* Copyright (c) 2012-2022 Andrew D. Zonenberg                                                                          *
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
	@brief Waveform data thread (data plane traffic only, no control plane SCPI)
 */
#include "ps6000d.h"
#include <string.h>

using namespace std;

volatile bool g_waveformThreadQuit = false;
float InterpolateTriggerTime(int16_t* buf);

vector<PICO_CHANNEL> g_channelIDs;

void WaveformServerThread()
{
	#ifdef __linux__
	pthread_setname_np(pthread_self(), "WaveformThread");
	#endif

	Socket client = g_dataSocket.Accept();
	LogVerbose("Client connected to data plane socket\n");

	if(!client.IsValid())
		return;
	if(!client.DisableNagle())
		LogWarning("Failed to disable Nagle on socket, performance may be poor\n");

	//Set up channel IDs
	for(size_t i=0; i<g_numChannels; i++)
		g_channelIDs.push_back((PICO_CHANNEL)i);
	for(size_t i=0; i<g_numDigitalPods; i++)
		g_channelIDs.push_back((PICO_CHANNEL)(PICO_PORT0 + i));

	map<size_t, int16_t*> waveformBuffers;
	size_t numSamples = 0;
	uint32_t numSamples_int = 0;
	uint16_t numchans;
	while(!g_waveformThreadQuit)
	{
		int16_t ready;
		{
			lock_guard<mutex> lock(g_mutex);
			if(g_pico_type == PICO6000A)
				ps6000aIsReady(g_hScope, &ready);
			else if(g_pico_type == PICO3000A)
				ps3000aIsReady(g_hScope, &ready);
		}

		if( (ready == 0) || (!g_triggerArmed) )
		{
			std::this_thread::sleep_for(std::chrono::microseconds(1000));
			continue;
		}

		size_t interval;
		map<size_t, bool> channelOn;
		bool msoPodEnabled[2];
		{
			lock_guard<mutex> lock(g_mutex);

			interval = g_sampleIntervalDuringArm;
			channelOn = g_channelOnDuringArm;
			msoPodEnabled[0] = g_msoPodEnabledDuringArm[0];
			msoPodEnabled[1] = g_msoPodEnabledDuringArm[1];

			//Stop the trigger
			PICO_STATUS status = PICO_OPERATION_FAILED;
			if(g_pico_type == PICO6000A)
				status = ps6000aStop(g_hScope);
			else if(g_pico_type == PICO3000A)
				status = ps3000aStop(g_hScope);
			if(PICO_OK != status)
				LogFatal("ps6000aStop failed (code 0x%x)\n", status);

			//Verify it's actually stopped

			//Set up buffers if needed
			if(g_memDepthChanged || waveformBuffers.empty())
			{
				LogTrace("Reallocating buffers\n");

				//Clear out old buffers
				for(auto ch : g_channelIDs)
				{
					if(g_pico_type == PICO6000A)
						ps6000aSetDataBuffer(g_hScope, ch, NULL,
							0, PICO_INT16_T, 0, PICO_RATIO_MODE_RAW, PICO_CLEAR_ALL);
					else if(g_pico_type == PICO3000A)
						ps3000aSetDataBuffer(g_hScope, (PS3000A_CHANNEL)ch, NULL,
							0, 0, PS3000A_RATIO_MODE_NONE);
				}

				//Clear out old buffers
				for(size_t i=0; i<g_channelIDs.size(); i++)
				{
					if(waveformBuffers[i])
					{
						delete[] waveformBuffers[i];
						waveformBuffers[i] = NULL;
					}
				}

				//Set up new ones
				//TODO: Only allocate memory if the channel is actually enabled
				for(size_t i=0; i<g_channelIDs.size(); i++)
				{
					//Allocate memory if needed
					waveformBuffers[i] = new int16_t[g_captureMemDepth];
					memset(waveformBuffers[i], 0x00, g_captureMemDepth * sizeof(int16_t));

					//Give it to the scope, removing any other buffer we might have
					auto ch = g_channelIDs[i];
					if(g_pico_type == PICO6000A)
						status = ps6000aSetDataBuffer(g_hScope, (PICO_CHANNEL)ch, waveformBuffers[i],
							g_captureMemDepth, PICO_INT16_T, 0, PICO_RATIO_MODE_RAW, PICO_ADD);
					else if(g_pico_type == PICO3000A)
						status = ps3000aSetDataBuffer(g_hScope, (PS3000A_CHANNEL)ch, waveformBuffers[i],
							g_captureMemDepth, 0, PS3000A_RATIO_MODE_NONE);
					if(status != PICO_OK)
						LogFatal("psXXXXSetDataBuffer for channel %d failed (code 0x%x)\n", ch, status);
				}

				g_memDepthChanged = false;
			}

			//Download the data from the scope
			numSamples = g_captureMemDepth;
			numSamples_int = g_captureMemDepth;
			int16_t overflow = 0;
			if(g_pico_type == PICO6000A)
				status = ps6000aGetValues(g_hScope, 0, &numSamples, 1, PICO_RATIO_MODE_RAW, 0, &overflow);
			else if(g_pico_type == PICO3000A)
				status = ps3000aGetValues(g_hScope, 0, &numSamples_int, 1, PS3000A_RATIO_MODE_NONE, 0, &overflow);
			if(status == PICO_NO_SAMPLES_AVAILABLE)
				continue; // state changed while mutex was unlocked?
			if(PICO_OK != status)
				LogFatal("psXXXXGetValues (code 0x%x)\n", status);

			//Figure out how many channels are active in this capture
			numchans = 0;
			for(size_t i=0; i<g_numChannels; i++)
			{
				if(g_channelOnDuringArm[i])
					numchans ++;
			}
			for(size_t i=0; i<g_numDigitalPods; i++)
			{
				if(g_msoPodEnabledDuringArm[i])
					numchans ++;
			}


		}

		//Do *not* hold mutex while sending data to the client
		//This can take a long time and we don't want to block the control channel
		{
			//Send the channel count to the client
			if(!client.SendLooped((uint8_t*)&numchans, sizeof(numchans)))
				break;

			//Send sample rate to the client
			if(!client.SendLooped((uint8_t*)&interval, sizeof(interval)))
				break;

			//TODO: send overflow flags to client

			//Interpolate trigger position if we're using an analog level trigger
			bool triggerIsAnalog = (g_triggerChannel < g_numChannels);
			float trigphase = 0;
			if(triggerIsAnalog)
				trigphase = InterpolateTriggerTime(waveformBuffers[g_triggerChannel]);

			//Send data for each channel to the client
			for(size_t i=0; i<g_channelIDs.size(); i++)
			{
				size_t header[2] = {i, numSamples};

				//Analog channels
				if((i < g_numChannels) && (channelOn[i]) )
				{
					//Send channel ID, scale, offset, and memory depth
					if(!client.SendLooped((uint8_t*)&header, sizeof(header)))
						break;

					float scale = g_roundedRange[i] / 32512;
					float offset = g_offsetDuringArm[i];
					float config[3] = {scale, offset, trigphase};
					if(!client.SendLooped((uint8_t*)&config, sizeof(config)))
						break;

					//Send the actual waveform data
					if(!client.SendLooped((uint8_t*)waveformBuffers[i], numSamples * sizeof(int16_t)))
						break;
				}

				//Digital channels
				else if( (i >= g_numChannels) && (msoPodEnabled[i - g_numChannels]) )
				{
					if(!client.SendLooped((uint8_t*)&header, sizeof(header)))
						break;
					if(!client.SendLooped((uint8_t*)&trigphase, sizeof(trigphase)))
						break;
					if(!client.SendLooped((uint8_t*)waveformBuffers[i], numSamples * sizeof(int16_t)))
						break;
				}
			}
		}

		//Need mutex here to update global state
		{
			lock_guard<mutex> lock(g_mutex);

			//Re-arm the trigger if doing repeating triggers
			if(g_triggerOneShot)
				g_triggerArmed = false;
			else
			{
				if(g_captureMemDepth != g_memDepth)
					g_memDepthChanged = true;

				//Restart
				StartCapture(false);
			}
		}
	}

	//Clean up temporary buffers
	for(auto it : waveformBuffers)
		delete[] it.second;
}

float InterpolateTriggerTime(int16_t* buf)
{
	if(g_triggerSampleIndex <= 0)
		return 0;

	float trigscale = g_roundedRange[g_triggerChannel] / 32512;
	float trigoff = g_offsetDuringArm[g_triggerChannel];

	float fa = buf[g_triggerSampleIndex-1] * trigscale + trigoff;
	float fb = buf[g_triggerSampleIndex] * trigscale + trigoff;

	//no need to divide by time, sample spacing is normalized to 1 timebase unit
	float slope = (fb - fa);
	float delta = g_triggerVoltage - fa;
	return delta / slope;
}
