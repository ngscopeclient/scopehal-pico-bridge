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
	@brief Declaration of PicoSCPIServer
 */

#ifndef PicoSCPIServer_h
#define PicoSCPIServer_h

#include "../../lib/scpi-server-tools/BridgeSCPIServer.h"

/**
	@brief SCPI server for managing control plane traffic to a single client
 */
class PicoSCPIServer : public BridgeSCPIServer
{
public:
	PicoSCPIServer(ZSOCKET sock);
	virtual ~PicoSCPIServer();

protected:
	virtual std::string GetMake();
	virtual std::string GetModel();
	virtual std::string GetSerial();
	virtual std::string GetFirmwareVersion();
	virtual size_t GetAnalogChannelCount();
	virtual std::vector<size_t> GetSampleRates();
	virtual std::vector<size_t> GetSampleDepths();

	virtual void OnCommand(
		const std::string& line,
		const std::string& subject,
		const std::string& cmd,
		const std::vector<std::string>& args);

	virtual bool OnQuery(
		const std::string& line,
		const std::string& subject,
		const std::string& cmd);

	virtual size_t GetChannelID(const std::string& subject);
};

#endif
