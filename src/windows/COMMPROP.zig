wPacketLength: WORD,
wPacketVersion: WORD,
dwServiceMask: DWORD,
dwReserved1: DWORD,
dwMaxTxQueue: DWORD,
dwMaxRxQueue: DWORD,
dwMaxBaud: DWORD,
dwProvSubType: DWORD,
dwProvCapabilities: DWORD,
dwSettableParams: DWORD,
dwSettableBaud: DWORD,
wSettableData: WORD,
wSettableStopParity: WORD,
dwCurrentTxQueue: DWORD,
dwCurrentRxQueue: DWORD,
dwProvSpec1: DWORD,
dwProvSpec2: DWORD,
wcProvChar: [1]WCHAR,

const std = @import("std");
const WORD = std.os.windows.WORD;
const DWORD = std.os.windows.DWORD;
const WCHAR = std.os.windows.WCHAR;