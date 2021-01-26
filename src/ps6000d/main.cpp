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
	@brief Program entry point
 */

#include "ps6000d.h"

using namespace std;

void help();

void help()
{
	fprintf(stderr,
			"ps6000d [general options] [logger options]\n"
			"\n"
			"  [general options]:\n"
			"    --help                        : this message...\n"
			"    --scpi-port port              : specifies the SCPI control plane port (default 5025)\n"
			"    --waveform-port port          : specifies the binary waveform data port (default 5026)\n"
			"\n"
			"  [logger options]:\n"
			"    levels: ERROR, WARNING, NOTICE, VERBOSE, DEBUG\n"
			"    --quiet|-q                    : reduce logging level by one step\n"
			"    --verbose                     : set logging level to VERBOSE\n"
			"    --debug                       : set logging level to DEBUG\n"
			"    --trace <classname>|          : name of class with tracing messages. (Only relevant when logging level is DEBUG.)\n"
			"            <classname::function>\n"
			"    --logfile|-l <filename>       : output log messages to file\n"
			"    --logfile-lines|-L <filename> : output log messages to file, with line buffering\n"
			"    --stdout-only                 : writes errors/warnings to stdout instead of stderr\n"
	);
}

string g_model;
string g_serial;
string g_fwver;

int16_t g_hScope = 0;

Socket g_scpiSocket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

int main(int argc, char* argv[])
{
	//Global settings
	Severity console_verbosity = Severity::NOTICE;

	//Parse command-line arguments
	uint16_t scpi_port = 5025;
	uint16_t waveform_port = 5026;
	for(int i=1; i<argc; i++)
	{
		string s(argv[i]);

		//Let the logger eat its args first
		if(ParseLoggerArguments(i, argc, argv, console_verbosity))
			continue;

		if(s == "--help")
		{
			help();
			return 0;
		}

		else if(s == "--scpi-port")
		{
			if(i+1 < argc)
				scpi_port = atoi(argv[++i]);
		}

		else if(s == "--waveform-port")
		{
			if(i+1 < argc)
				waveform_port = atoi(argv[++i]);
		}

		else
		{
			fprintf(stderr, "Unrecognized command-line argument \"%s\", use --help\n", s.c_str());
			return 1;
		}
	}

	//Set up logging
	g_log_sinks.emplace(g_log_sinks.begin(), new ColoredSTDLogSink(console_verbosity));

	//For now, open the first instrument we can find.
	//TODO: implement device selection logic
	LogNotice("Looking for a PicoScope 6000 series instrument to open...\n");
	auto status = ps6000aOpenUnit(&g_hScope, NULL, PICO_DR_8BIT);
	if(PICO_OK != status)
	{
		LogError("Failed to open unit (code %d)\n", status);
		return 1;
	}

	//See what we got
	LogNotice("Successfully opened instrument\n");
	{
		LogIndenter li;

		char buf[128];
		int16_t required = 0;
		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_DRIVER_VERSION);
		if(status == PICO_OK)
			LogVerbose("Driver version:   %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_USB_VERSION);
		if(status == PICO_OK)
			LogVerbose("USB version:      %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_HARDWARE_VERSION);
		if(status == PICO_OK)
			LogVerbose("Hardware version: %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_VARIANT_INFO);
		if(status == PICO_OK)
		{
			LogVerbose("Variant info:     %s\n", buf);
			g_model = buf;
		}

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_BATCH_AND_SERIAL);
		if(status == PICO_OK)
		{
			LogVerbose("Batch/serial:     %s\n", buf);
			g_serial = buf;
		}

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_CAL_DATE);
		if(status == PICO_OK)
			LogVerbose("Cal date:         %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_KERNEL_VERSION);
		if(status == PICO_OK)
			LogVerbose("Kernel ver:       %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_DIGITAL_HARDWARE_VERSION);
		if(status == PICO_OK)
			LogVerbose("Digital HW ver:   %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_ANALOGUE_HARDWARE_VERSION);
		if(status == PICO_OK)
			LogVerbose("Analog HW ver:    %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_FIRMWARE_VERSION_1);
		if(status == PICO_OK)
		{
			LogVerbose("FW ver 1:         %s\n", buf);
			g_fwver = buf;
		}

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_FIRMWARE_VERSION_2);
		if(status == PICO_OK)
			LogVerbose("FW ver 2:         %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_FIRMWARE_VERSION_3);
		if(status == PICO_OK)
			LogVerbose("FW ver 3:         %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_FRONT_PANEL_FIRMWARE_VERSION);
		if(status == PICO_OK)
			LogVerbose("Front panel FW:   %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_MAC_ADDRESS);
		if(status == PICO_OK)
			LogVerbose("MAC address:      %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_DRIVER_PATH);
		if(status == PICO_OK)
			LogVerbose("Driver path:      %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_SHADOW_CAL);
		if(status == PICO_OK)
			LogVerbose("Shadow cal:       %s\n", buf);

		status = ps6000aGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_IPP_VERSION);
		if(status == PICO_OK)
			LogVerbose("IPP version:      %s\n", buf);
	}

	//TODO: Launch the data plane socket server

	//Launch the control plane socket server
	g_scpiSocket.Bind(scpi_port);
	g_scpiSocket.Listen();
	thread scpiThread(ScpiServerThread);

	//TODO: proper clean shutdown with ^C

	//Wait for threads to terminate
	scpiThread.join();

	//Done
	ps6000aCloseUnit(g_hScope);
	return 0;
}
