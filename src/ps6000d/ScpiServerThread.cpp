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

		EXIT
			Terminates the connection
 */

#include "ps6000d.h"

using namespace std;

bool ScpiSend(Socket& sock, const string& cmd);
string ScpiRecv(Socket& sock);
void ParseScpiLine(const string& line, string& subject, string& cmd, bool& query, vector<string>& args);

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
string ScpiRecv(Socket& sock)
{
	char tmp = ' ';
	string ret;
	while(true)
	{
		if(!sock.RecvLooped((unsigned char*)&tmp, 1))
			return "EXIT";
		if( (tmp == '\n') || ( (tmp == ';') ) )
			break;
		else
			ret += tmp;
	}
	return ret;
}

/**
	@brief Main socket server
 */
void ScpiServerThread()
{
	while(true)
	{
		Socket client = g_scpiSocket.Accept();
		if(!client.IsValid())
			break;
		if(!client.DisableNagle())
			LogWarning("Failed to disable Nagle on socket, performanec may be poor\n");

		LogVerbose("Client connected\n");

		//Main command loop
		string cmd;
		bool query;
		string subject;
		vector<string> args;
		while(true)
		{
			string line = ScpiRecv(client);
			ParseScpiLine(line, subject, cmd, query, args);

			//Quit
			if(cmd == "EXIT")
				break;

			//Read ID code
			else if( (cmd == "*IDN") && query )
				ScpiSend(client, string("Pico Technology,") + g_model + "," + g_serial + "," + g_fwver);

			//Unknown
			else
			{
				LogDebug("Unrecognized command received: %s\n", line.c_str());
				LogIndenter li;
				LogDebug("Subject: %s\n", subject.c_str());
				LogDebug("Command: %s\n", cmd.c_str());
				if(query)
					LogDebug("Query\n");
				else
					LogDebug("Not query\n");
				for(auto arg : args)
					LogDebug("Arg: %s\n", arg.c_str());
			}
		}

		LogVerbose("Client disconnected\n");
	}
}

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
