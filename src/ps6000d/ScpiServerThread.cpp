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

		[chan]:ON
			Turns the channel on

		[chan]:OFF
			Turns the channel off

		[chan]:COUP [DC1M|AC1M|DC50]
			Sets channel coupling

		[chan]:OFFS [num]
			Sets channel offset to num volts

		[chan]:RANGE [num]
			Sets channel full-scale range to num volts

		RATES?
			Returns a comma separated list of sampling rates (in femtoseconds)

		START
			Arms the trigger

		STOP
			Disarms the trigger

		CONNECT
			Waits for connection on the data plane socket

		EXIT
			Terminates the connection
 */

#include "ps6000d.h"
#include <string.h>

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
size_t g_memDepth = 100000;

//Copy of state at timestamp of last arm event
map<size_t, bool> g_channelOnDuringArm;
size_t g_captureMemDepth = 0;

volatile bool g_triggerArmed = false;

void UpdateChannel(size_t chan);

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

	size_t maxTimebases = 100;

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
			size_t channelId = subject[0] - 'A';
			channelId = min(channelId, g_numChannels);

			if(query)
			{
				//Read ID code
				if(cmd == "*IDN")
					ScpiSend(client, string("Pico Technology,") + g_model + "," + g_serial + "," + g_fwver);

				//Get number of channels
				else if(cmd == "CHANS")
					ScpiSend(client, to_string(g_numChannels));

				//Get memory depth
				else if(cmd == "RATES")
				{
					string ret = "";

					//Enumerate timebases
					for(size_t i=0; i<maxTimebases; i++)
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

				else
					LogDebug("Unrecognized query received: %s\n", line.c_str());
			}

			else if(cmd == "EXIT")
				break;

			else if(cmd == "ON")
			{
				g_channelOn[channelId] = true;
				UpdateChannel(channelId);
			}
			else if(cmd == "OFF")
			{
				g_channelOn[channelId] = false;
				UpdateChannel(channelId);
			}

			else if(cmd == "COUP")
			{
				if(args[0] == "DC1M")
					g_coupling[channelId] = PICO_DC;
				else if(args[0] == "AC1M")
					g_coupling[channelId] = PICO_AC;
				else if(args[0] == "DC50")
					g_coupling[channelId] = PICO_DC_50OHM;

				UpdateChannel(channelId);
			}

			else if(cmd == "OFFS")
			{
				g_offset[channelId] = stod(args[0]);
				UpdateChannel(channelId);
			}

			else if(cmd == "RANGE")
			{
				auto range = stod(args[0]);

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
			}

			else if(cmd == "START")
			{
				if(g_triggerArmed)
				{
					LogVerbose("Ignoring START command because trigger is already armed\n");
					continue;
				}

				LogDebug("arming trigger\n");

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

				g_channelOnDuringArm = g_channelOn;
				g_captureMemDepth = g_memDepth;

				//Start the capture
				auto status = ps6000aRunBlock(g_hScope, g_memDepth/2, g_memDepth/2, 2, NULL, 0, NULL, NULL);
				if(status != PICO_OK)
					LogFatal("ps6000aRunBlock failed, code %d\n", status);

				g_triggerArmed = true;
			}

			else if(cmd == "STOP")
			{
				ps6000aStop(g_hScope);
				g_triggerArmed = false;
			}

			//TODO: range
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
		if(line[i] == ':')
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
			g_coupling[chan], g_range[chan], g_offset[chan], g_bandwidth[chan]);
	}
	else
		ps6000aSetChannelOff(g_hScope, (PICO_CHANNEL)chan);
}
