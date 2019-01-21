#pragma once

#pragma pack(push, 1)
struct CmdMaster
{
	uint8_t cmd;
};
#pragma pack(pop)

struct CmdSlave
{
	uint8_t data1;
	uint8_t data2;
};

