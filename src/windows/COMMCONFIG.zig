dwSize: DWORD,
wVersion: WORD,
wReserved: WORD,
dcb: DCB,
dwProviderSubType: DWORD,
dwProviderOffset: DWORD,
dwProviderSize: DWORD,
wcProviderData: [1]WCHAR,

const std = @import("std");
const DCB = @import("DCB.zig");
const DWORD = std.os.windows.DWORD;
const WORD = std.os.windows.WORD;
const WCHAR = std.os.windows.WCHAR;