#pragma once
#include <cstdint>   // faltaba: uint32_t

namespace go2rc
{
	enum class Button : uint32_t
	{
		NONE = 0x0000, R1 = 0x0001, L1 = 0x0002, START = 0x0004, SELECT = 0x0008,
		R2 = 0x0010, L2 = 0x0020, F1 = 0x0040, F3 = 0x0080,
		A = 0x0100, B = 0x0200, X = 0x0400, Y = 0x0800,
		UP = 0x1000, RIGHT = 0x2000, DOWN = 0x4000, LEFT = 0x8000,
	};

	inline bool pressed(uint32_t keys, Button b) { return (keys & static_cast<uint32_t>(b)) != 0; }
}

