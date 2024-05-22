DCBlength: DWORD,
BaudRate: DWORD,
flags: u32,
wReserved: WORD,
XonLin: WORD,
XoffLim: WORD,
ByteSize: BYTE,
Parity: BYTE,
StopBits: BYTE,
XonChar: c_char,
XoffChar: c_char,
ErrorChar: c_char,
EofChar: c_char,
EvtChar: c_char,
wReserved1: WORD,

const std = @import("std");
const DWORD = std.os.windows.DWORD;
const WORD = std.os.windows.WORD;
const BYTE = std.os.windows.BYTE;
