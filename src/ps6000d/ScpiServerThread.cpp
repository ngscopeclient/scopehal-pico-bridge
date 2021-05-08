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

	SCPI commands supported:

		*IDN?
			Returns a standard SCPI instrument identification string

		CHANS?
			Returns the number of channels on the instrument.

		[1|2]D:PRESENT?
			Returns 1 = MSO pod present, 0 = MSO pod not present

		[chan]:COUP [DC1M|AC1M|DC50]
			Sets channel coupling

		[chan]:OFF
			Turns the channel off

		[chan]:OFFS [num]
			Sets channel offset to num volts

		[chan]:ON
			Turns the channel on

		[chan]:RANGE [num]
			Sets channel full-scale range to num volts

		BITS [num]
			Sets ADC bit depth

		DEPTH [num]
			Sets memory depth

		DEPTHS?
			Returns the set of available memory depths

		EXIT
			Terminates the connection

		RATE [num]
			Sets sample rate

		RATES?
			Returns a comma separated list of sampling rates (in femtoseconds)

		SINGLE
			Arms the trigger in one-shot mode

		START
			Arms the trigger

		STOP
			Disarms the trigger

		TRIG:DELAY [delay]
			Sets trigger delay (in fs)

		TRIG:EDGE:DIR [direction]
			Sets trigger direction. Legal values are RISING, FALLING, or ANY.

		TRIG:LEV [level]
			Selects trigger level (in volts)

		TRIG:SOU [chan]
			Selects the channel as the trigger source
 */

#include "ps6000d.h"
#include <string.h>
#include <math.h>

using namespace std;

bool ScpiSend(Socket& sock, const string& cmd);
bool ScpiRecv(Socket& sock, string& str);
void ParseScpiLine(const string& line, string& subject, string& cmd, bool& query, vector<string>& args);

//Channel state
map<size_t, bool> g_channelOn;
map<size_t, PICO_COUPLING> g_coupling;
map<size_t, PICO_CONNECT_PROBE_RANGE> g_range;
map<size_t, double> g_roundedRange;
map<size_t, double> g_offset;
map<size_t, PICO_BANDWIDTH_LIMITER> g_bandwidth;
size_t g_memDepth = 1000000;
int64_t g_sampleInterval = 0;	//in fs

//Copy of state at timestamp of last arm event
map<size_t, bool> g_channelOnDuringArm;
int64_t g_sampleIntervalDuringArm = 0;
size_t g_captureMemDepth = 0;
map<size_t, double> g_offsetDuringArm;

uint32_t g_timebase = 0;

bool g_triggerArmed = false;
bool g_triggerOneShot = false;
bool g_memDepthChanged = false;

//Trigger state (for now, only simple single-channel trigger supported)
uint64_t g_triggerDelay = 0;
PICO_THRESHOLD_DIRECTION g_triggerDirection = PICO_RISING;
float g_triggerVoltage = 0;
size_t g_triggerChannel = 0;
size_t g_triggerSampleIndex;

//Thresholds for MSO pods
int16_t g_msoPodThreshold[2][8] = { {0}, {0} };
PICO_DIGITAL_PORT_HYSTERESIS g_msoHysteresis[2] = {PICO_NORMAL_100MV, PICO_NORMAL_100MV};
bool g_msoPodEnabled[2] = {false};

void UpdateTrigger();
void UpdateChannel(size_t chan);

std::mutex g_mutex;

/**
	@brief Sends a SCPI reply (terminated by newline)
 */
bool ScpiSend(Socket& sock, const string& cmd)
{
	string tempbuf = cmd + "\n";
	return sock.SendLooped((unsigned char*)tempbuf.c_str(), tempbuf.length());
}

/**
	@brief Reads a SCPI command (terminated by newline or semicolon)
 */
bool ScpiRecv(Socket& sock, string& str)
{
	int sockid = sock;

	char tmp = ' ';
	str = "";
	while(true)
	{
		if(1 != recv(sockid, &tmp, 1, MSG_WAITALL))
			return false;

		if( (tmp == '\n') || ( (tmp == ';') ) )
			break;
		else
			str += tmp;
	}

	return true;
}

/**
	@brief Main socket server
 */
void ScpiServerThread()
{
	for(size_t i=0; i<g_numChannels; i++)
	{
		g_channelOn[i] = false;
		g_coupling[i] = PICO_DC;
		g_range[i] = PICO_X1_PROBE_1V;
		g_offset[i] = 0;
		g_bandwidth[i] = PICO_BW_FULL;
	}

	//Push initial trigger config
	{
		lock_guard<mutex> lock(g_mutex);
		UpdateTrigger();
	}

	while(true)
	{
		Socket client = g_scpiSocket.Accept();
		Socket dataClient(-1);
		LogVerbose("Client connected to control plane socket\n");

		if(!client.IsValid())
			break;
		if(!client.DisableNagle())
			LogWarning("Failed to disable Nagle on socket, performance may be poor\n");

		thread dataThread(WaveformServerThread);

		//Main command loop
		string line;
		string cmd;
		bool query;
		string subject;
		vector<string> args;
		while(true)
		{
			if(!ScpiRecv(client, line))
				break;
			ParseScpiLine(line, subject, cmd, query, args);

			//Extract channel ID from subject and clamp bounds
			size_t channelId = 0;
			bool channelIsDigital = false;
			if(isalpha(subject[0]))
			{
				channelId = min(static_cast<size_t>(subject[0] - 'A'), g_numChannels);
				channelIsDigital = false;
			}
			else if(isdigit(subject[0]))
			{
				channelId = min(subject[0] - '0', 2) - 1;
				channelIsDigital = true;
			}

			if(query)
			{
				//Read ID code
				if(cmd == "*IDN")
					ScpiSend(client, string("Pico Technology,") + g_model + "," + g_serial + "," + g_fwver);

				//Get number of channels
				else if(cmd == "CHANS")
					ScpiSend(client, to_string(g_numChannels));

				//Get legal sample rates for the current configuration
				else if(cmd == "RATES")
				{
					string ret = "";

					lock_guard<mutex> lock(g_mutex);

					//Enumerate timebases
					//Don't report every single legal timebase as there's way too many, the list box would be huge!
					//Report the first nine, then go to larger steps
					size_t vec[] =
					{
						0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
						14, 29, 54, 104, 129, 254, 504, 629, 1254, 2504, 3129, 5004, 6254, 15629, 31254,
						62504, 156254, 312504, 625004, 1562504
					};
					for(auto i : vec)
					{
						double intervalNs;
						size_t maxSamples;
						if(PICO_OK == ps6000aGetTimebase(g_hScope, i, 1, &intervalNs, &maxSamples, 0))
						{
							size_t intervalFs = intervalNs * 1e6f;
							ret += to_string(intervalFs) + ",";
						}
					}
					ScpiSend(client, ret);
				}

				//Get memory depths
				else if(cmd == "DEPTHS")
				{
					string ret = "";

					lock_guard<mutex> lock(g_mutex);
					double intervalNs;
					size_t maxSamples;

					//Ask for max memory depth at 1.25 Gsps. Why does legal memory depend on sample rate?
					if(PICO_OK == ps6000aGetTimebase(g_hScope, 2, 1, &intervalNs, &maxSamples, 0))
					{
						//Seems like there's no restrictions on actual memory depth other than an upper bound.
						//To keep things simple, report 1-2-5 series from 10K samples up to the actual max depth

						for(size_t base = 10000; base < maxSamples; base *= 10)
						{
							const size_t muls[] = {1, 2, 5};
							for(auto m : muls)
							{
								size_t depth = m * base;
								if(depth < maxSamples)
									ret += to_string(depth) + ",";
							}
						}

						ret += to_string(maxSamples) + ",";
					}

					ScpiSend(client, ret);
				}

				else if(cmd == "PRESENT")
				{
					lock_guard<mutex> lock(g_mutex);

					//There's no API to test for presence of a MSO pod without trying to enable it.
					//If no pod is present, this call will return PICO_NO_MSO_POD_CONNECTED.
					PICO_CHANNEL podId = (PICO_CHANNEL)(PICO_PORT0 + channelId);
					auto status = ps6000aSetDigitalPortOn(
						g_hScope,
						podId,
						g_msoPodThreshold[channelId],
						8,
						g_msoHysteresis[channelId]);

					if(status == PICO_NO_MSO_POD_CONNECTED)
						ScpiSend(client, "0");

					//The pod is here. If we don't need it on, shut it back off
					else
					{
						if(!g_msoPodEnabled[channelId])
							ps6000aSetDigitalPortOff(g_hScope, podId);

						ScpiSend(client, "1");
					}
				}

				else
					LogDebug("Unrecognized query received: %s\n", line.c_str());
			}

			else if(cmd == "EXIT")
				break;

			else if(cmd == "ON")
			{
				lock_guard<mutex> lock(g_mutex);

				if(channelIsDigital)
				{
					LogDebug("Enabling MSO pod %zu\n", channelId);
				}
				else
				{
					g_channelOn[channelId] = true;
					UpdateChannel(channelId);
				}

			}
			else if(cmd == "OFF")
			{
				lock_guard<mutex> lock(g_mutex);

				if(channelIsDigital)
				{
					LogDebug("Disabling MSO pod %zu\n", channelId);
				}
				else
				{
					g_channelOn[channelId] = false;
					UpdateChannel(channelId);
				}
			}

			else if( (cmd == "BITS") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				ps6000aStop(g_hScope);

				//Even though we didn't actually change memory, apparently calling ps6000aSetDeviceResolution
				//will invalidate the existing buffers and make ps6000aGetValues() fail with PICO_BUFFERS_NOT_SET.
				g_memDepthChanged = true;

				int bits = stoi(args[0].c_str());
				switch(bits)
				{
					case 8:
						ps6000aSetDeviceResolution(g_hScope, PICO_DR_8BIT);
						break;

					case 10:
						ps6000aSetDeviceResolution(g_hScope, PICO_DR_10BIT);
						break;

					case 12:
						ps6000aSetDeviceResolution(g_hScope, PICO_DR_12BIT);
						break;

					default:
						LogError("User requested invalid resolution (%d bits)\n", bits);
				}

				if(g_triggerArmed)
					StartCapture(false);
			}

			else if( (cmd == "COUP") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				if(args[0] == "DC1M")
					g_coupling[channelId] = PICO_DC;
				else if(args[0] == "AC1M")
					g_coupling[channelId] = PICO_AC;
				else if(args[0] == "DC50")
					g_coupling[channelId] = PICO_DC_50OHM;

				UpdateChannel(channelId);
			}

			else if( (cmd == "OFFS") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				double requestedOffset = stod(args[0]);

				//Clamp to allowed range
				double maxoff;
				double minoff;
				ps6000aGetAnalogueOffsetLimits(g_hScope, g_range[channelId], g_coupling[channelId], &maxoff, &minoff);
				requestedOffset = min(maxoff, requestedOffset);
				requestedOffset = max(minoff, requestedOffset);

				g_offset[channelId] = requestedOffset;
				UpdateChannel(channelId);
			}

			else if( (cmd == "RANGE") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				auto range = stod(args[0]);

				//If 50 ohm coupling, cap hardware voltage range to 5V
				if(g_coupling[channelId] == PICO_DC_50OHM)
					range = min(range, 5.0);

				if(range > 100)
				{
					g_range[channelId] = PICO_X1_PROBE_200V;
					g_roundedRange[channelId] = 200;
				}
				else if(range > 50)
				{
					g_range[channelId] = PICO_X1_PROBE_100V;
					g_roundedRange[channelId] = 100;
				}
				else if(range > 20)
				{
					g_range[channelId] = PICO_X1_PROBE_50V;
					g_roundedRange[channelId] = 50;
				}
				else if(range > 10)
				{
					g_range[channelId] = PICO_X1_PROBE_20V;
					g_roundedRange[channelId] = 20;
				}
				else if(range > 5)
				{
					g_range[channelId] = PICO_X1_PROBE_10V;
					g_roundedRange[channelId] = 10;
				}
				else if(range > 2)
				{
					g_range[channelId] = PICO_X1_PROBE_5V;
					g_roundedRange[channelId] = 5;
				}
				else if(range > 1)
				{
					g_range[channelId] = PICO_X1_PROBE_2V;
					g_roundedRange[channelId] = 2;
				}
				else if(range > 0.5)
				{
					g_range[channelId] = PICO_X1_PROBE_1V;
					g_roundedRange[channelId] = 1;
				}
				else if(range > 0.2)
				{
					g_range[channelId] = PICO_X1_PROBE_500MV;
					g_roundedRange[channelId] = 0.5;
				}
				else if(range > 0.1)
				{
					g_range[channelId] = PICO_X1_PROBE_200MV;
					g_roundedRange[channelId] = 0.2;
				}
				else if(range >= 0.05)
				{
					g_range[channelId] = PICO_X1_PROBE_100MV;
					g_roundedRange[channelId] = 0.1;
				}
				else if(range >= 0.02)
				{
					g_range[channelId] = PICO_X1_PROBE_50MV;
					g_roundedRange[channelId] = 0.05;
				}
				else if(range >= 0.01)
				{
					g_range[channelId] = PICO_X1_PROBE_20MV;
					g_roundedRange[channelId] = 0.02;
				}
				else
				{
					g_range[channelId] = PICO_X1_PROBE_10MV;
					g_roundedRange[channelId] = 0.01;
				}

				UpdateChannel(channelId);

				//Update trigger if this is the trigger channel.
				//Trigger is digital and threshold is specified in ADC counts.
				//We want to maintain constant trigger level in volts, not ADC counts.
				if(g_triggerChannel == channelId)
					UpdateTrigger();
			}

			else if( (cmd == "RATE") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);

				//Convert sample rate to sample period
				auto rate = stoull(args[0]);
				g_sampleInterval = 1e15 / rate;
				double period_ns = 1e9 / rate;

				//Find closest timebase setting
				double clkdiv = period_ns / 0.2;
				int timebase;
				if(period_ns < 5)
					timebase = round(log(clkdiv)/log(2));
				else
					timebase = round(clkdiv) + 4;

				g_timebase = timebase;
			}

			else if( (cmd == "DEPTH") && (args.size() == 1) )
			{
				lock_guard<mutex> lock(g_mutex);
				g_memDepth = stoull(args[0]);
			}

			else if( (cmd == "START") || (cmd == "SINGLE") )
			{
				lock_guard<mutex> lock(g_mutex);

				if(g_triggerArmed)
				{
					LogVerbose("Ignoring START command because trigger is already armed\n");
					continue;
				}

				bool anyChannels = false;
				for(size_t i=0; i<g_numChannels; i++)
				{
					if(g_channelOn[i])
						anyChannels = true;
				}

				if(!anyChannels)
				{
					LogVerbose("Ignoring START command because no channels are active\n");
					continue;
				}

				if(g_captureMemDepth != g_memDepth)
					g_memDepthChanged = true;

				//Start the capture
				StartCapture(false);
				g_triggerOneShot = (cmd == "SINGLE");
			}

			else if(cmd == "STOP")
			{
				lock_guard<mutex> lock(g_mutex);

				ps6000aStop(g_hScope);
				g_triggerArmed = false;
			}

			else if(subject == "TRIG")
			{
				if( (cmd == "EDGE:DIR") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					if(args[0] == "RISING")
						g_triggerDirection = PICO_RISING;
					else if(args[0] == "FALLING")
						g_triggerDirection = PICO_FALLING;
					else if(args[0] == "ANY")
						g_triggerDirection = PICO_RISING_OR_FALLING;

					UpdateTrigger();
				}

				else if( (cmd == "LEV") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					g_triggerVoltage = stof(args[0]);
					UpdateTrigger();
				}

				else if( (cmd == "SOU") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);
					g_triggerChannel = args[0][0] - 'A';

					UpdateTrigger();
				}

				else if( (cmd == "DELAY") && (args.size() == 1) )
				{
					lock_guard<mutex> lock(g_mutex);

					g_triggerDelay = stoull(args[0]);
					UpdateTrigger();
				}

				else
				{
					LogDebug("Unrecognized trigger command received: %s\n", line.c_str());
					LogIndenter li;
					LogDebug("Command: %s\n", cmd.c_str());
					for(auto arg : args)
						LogDebug("Arg: %s\n", arg.c_str());
				}
			}

			//TODO: bandwidth limiter

			//Unknown
			else
			{
				LogDebug("Unrecognized command received: %s\n", line.c_str());
				LogIndenter li;
				LogDebug("Subject: %s\n", subject.c_str());
				LogDebug("Command: %s\n", cmd.c_str());
				for(auto arg : args)
					LogDebug("Arg: %s\n", arg.c_str());
			}
		}

		LogVerbose("Client disconnected\n");

		//Disable all channels when a client disconnects to put the scope in a "safe" state
		for(auto it : g_channelOn)
			ps6000aSetChannelOff(g_hScope, (PICO_CHANNEL)it.first);
		for(int i=0; i<2; i++)
		{
			ps6000aSetDigitalPortOff(g_hScope, (PICO_CHANNEL)(PICO_PORT0 + i));
			g_msoPodEnabled[i] = false;
		}

		g_waveformThreadQuit = true;
		dataThread.join();
		g_waveformThreadQuit = false;
	}
}

/**
	@brief Parses an incoming SCPI command
 */
void ParseScpiLine(const string& line, string& subject, string& cmd, bool& query, vector<string>& args)
{
	//Reset fields
	query = false;
	subject = "";
	cmd = "";
	args.clear();

	string tmp;
	bool reading_cmd = true;
	for(size_t i=0; i<line.length(); i++)
	{
		//If there's no colon in the command, the first block is the command.
		//If there is one, the first block is the subject and the second is the command.
		//If more than one, treat it as freeform text in the command.
		if( (line[i] == ':') && subject.empty() )
		{
			subject = tmp;
			tmp = "";
			continue;
		}

		//Detect queries
		if(line[i] == '?')
		{
			query = true;
			continue;
		}

		//Comma delimits arguments, space delimits command-to-args
		if(!(isspace(line[i]) && cmd.empty()) && line[i] != ',')
		{
			tmp += line[i];
			continue;
		}

		//merge multiple delimiters into one delimiter
		if(tmp == "")
			continue;

		//Save command or argument
		if(reading_cmd)
			cmd = tmp;
		else
			args.push_back(tmp);

		reading_cmd = false;
		tmp = "";
	}

	//Stuff left over at the end? Figure out which field it belongs in
	if(tmp != "")
	{
		if(cmd != "")
			args.push_back(tmp);
		else
			cmd = tmp;
	}
}

/**
	@brief Pushes channel configuration to the instrument
 */
void UpdateChannel(size_t chan)
{
	if(g_channelOn[chan])
	{
		ps6000aSetChannelOn(g_hScope, (PICO_CHANNEL)chan,
			g_coupling[chan], g_range[chan], -g_offset[chan], g_bandwidth[chan]);

		//We use software triggering based on raw ADC codes.
		//Any time we change the frontend configuration on the trigger channel, it has to be reconfigured.
		//TODO: handle multi-input triggers
		if(chan == g_triggerChannel)
			UpdateTrigger();
	}
	else
		ps6000aSetChannelOff(g_hScope, (PICO_CHANNEL)chan);
}

/**
	@brief Pushes trigger configuration to the instrument
 */
void UpdateTrigger()
{
	//Convert threshold from volts to ADC counts
	float offset = g_offset[g_triggerChannel];
	float scale = g_roundedRange[g_triggerChannel] / 32512;
	if(scale == 0)
		scale = 1;
	float trig_code = (g_triggerVoltage - offset) / scale;
	//LogDebug("UpdateTrigger: trig_code = %.0f for %f V, scale=%f\n", round(trig_code), g_triggerVoltage, scale);

	//TODO: Convert delay from fs to native units (samples? ps?)
	uint64_t delay = 0;

	ps6000aSetSimpleTrigger(
		g_hScope,
		1,
		(PICO_CHANNEL)g_triggerChannel,
		round(trig_code),
		g_triggerDirection,
		delay,
		0);

	if(g_triggerArmed)
		StartCapture(true);
}

void StartCapture(bool stopFirst)
{
	g_offsetDuringArm = g_offset;
	g_channelOnDuringArm = g_channelOn;
	g_captureMemDepth = g_memDepth;
	g_sampleIntervalDuringArm = g_sampleInterval;
	g_triggerSampleIndex = g_memDepth/2;

	//TODO: implement g_triggerDelay

	if(stopFirst)
		ps6000aStop(g_hScope);

	auto status = ps6000aRunBlock(g_hScope, g_memDepth/2, g_memDepth/2, g_timebase, NULL, 0, NULL, NULL);

	//not sure why this happens...
	if(status == PICO_HARDWARE_CAPTURING_CALL_STOP)
	{
		LogWarning("Got PICO_HARDWARE_CAPTURING_CALL_STOP (but scope should have been stopped already)\n");
		ps6000aStop(g_hScope);
		status = ps6000aRunBlock(g_hScope, g_memDepth/2, g_memDepth/2, g_timebase, NULL, 0, NULL, NULL);
	}

	if(status != PICO_OK)
		LogFatal("ps6000aRunBlock failed, code %d / 0x%x\n", status, status);

	g_triggerArmed = true;
}
