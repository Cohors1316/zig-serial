const std = @import("std");
const serial = @import("../common_serial.zig");
const SerialPort = @import("../SerialPort.zig");
const File = std.fs.File;
const Pins = serial.Pins;
const Buffers = serial.Buffers;

pub fn setPins(file: File, pins: Pins) !void {
    if (pins.rts) |rts| {
        if (EscapeCommFunction(file.hanle, if (rts) 3 else 4) == 0)
            return error.ControlPins;
    }
    if (pins.dtr) |dtr| {
        if (EscapeCommFunction(file.handle, if (dtr) 5 else 6) == 0)
            return error.ControlPins;
    }
}

pub fn flush(file: File, buffers: Buffers) !void {
    const mode = switch (buffers) {
        .input => 0x0008,
        .output => 0x0004,
        .both => 0x0008 | 0x0004,
    };
    if (PurgeComm(file.handle, mode) == 0)
        return error.PurgeBuffers;
}


pub fn open(self: *SerialPort) !void {
    if (self.p.file) return;
    var buffer: [std.fs.max_path_bytes]u8 = undefined;
    const path = try std.fmt.bufPrint(&buffer, "\\\\.\\", .{self.p.name});
    self.p.file = try std.fs.openFileAbsolute(path, .{.mode = .read_write});
    try configure(self.file, self.config);
}

pub fn configure(file: std.fs.File, config: common_serial.Config) !void {
    var dcb = std.mem.zeroes(DCB);
    dcb.DCBlength = @sizeOf(DCB);
    if (GetCommState(file.handle, &dcb) == 0) return error.configure;
    dcb.BaudRate = config.baud_rate;
    const dcb_flags = DCBFlags{
        .fBinary = 1,
        .fParity = @intFromBool(config.parity != .none),
        .fOutxCtsFlow = @intFromBool(config.handshake == .hardware),
        .fOutX = @intFromBool(config.handshake == .software),
        .fInX = @intFromBool(config.handshake == .software),
        .fRtsControl = @intFromBool(config.handshake == .hardware),
    };
    dcb.flags = dcb_flags.toNumeric();
    dcb.wReserved = 0;
    dcb.wReserved1 = 0;
    dcb.XonChar = 0x11;
    dcb.XoffChar = 0x13;
    dcb.StopBits = switch (config.stop_bits) {
        .one => 0,
        .two => 2,
    };
    dcb.ByteSize = switch (config.word_size) {
        .five => 5,
        .six => 6,
        .seven => 7,
        .eight => 8,
    };
    dcb.Parity = switch (config.parity) {
        .none => 0,
        .odd => 1,
        .even => 2,
        .mark => 3,
        .space => 4,
    };
    if (SetCommState(file.handle, &dcb) == 0) return error.configure;
}

const DCBFlags = struct {
    fBinary: u1 = 1, // u1
    fParity: u1 = 0, // u1
    fOutxCtsFlow: u1 = 0, // u1
    fOutxDsrFlow: u1 = 0, // u1
    fDtrControl: u2 = 0, // u2
    fDsrSensitivity: u1 = 0, // u1
    fTXContinueOnXoff: u1 = 0, // u1
    fOutX: u1 = 0, // u1
    fInX: u1 = 0, // u1
    fErrorChar: u1 = 0, // u1
    fNull: u1 = 0, // u1
    fRtsControl: u2 = 0, // u2
    fAbortOnError: u1 = 0, // u1
    fDummy2: u17 = 0, // u17

    pub fn toNumeric(self: DCBFlags) u32 {
        var value: u32 = 0;
        value += @as(u32, self.fBinary) << 0; // u1
        value += @as(u32, self.fParity) << 1; // u1
        value += @as(u32, self.fOutxCtsFlow) << 2; // u1
        value += @as(u32, self.fOutxDsrFlow) << 3; // u1
        value += @as(u32, self.fDtrControl) << 4; // u2
        value += @as(u32, self.fDsrSensitivity) << 6; // u1
        value += @as(u32, self.fTXContinueOnXoff) << 7; // u1
        value += @as(u32, self.fOutX) << 8; // u1
        value += @as(u32, self.fInX) << 9; // u1
        value += @as(u32, self.fErrorChar) << 10; // u1
        value += @as(u32, self.fNull) << 11; // u1
        value += @as(u32, self.fRtsControl) << 12; // u2
        value += @as(u32, self.fAbortOnError) << 14; // u1
        value += @as(u32, self.fDummy2) << 15; // u17
        return value;
    }
};

const WINAPI = std.os.windows.WINAPI;
const COMMCONFIG = @import("COMMCONFIG.zig");
const COMMPROP = @import("COMMPROP.zig");
const CSTR = []const u8;
const COMMTIMEOUTS = @import("COMMTIMEOUTS.zig");
const DCB = @import("DCB.zig");
const COMSTAT = @import("COMSTAT.zig");
const HANDLE = std.os.windows.HANDLE;
const DWORD = std.os.windows.DWORD;
const BOOL = std.os.windows.BOOL;
const HWND = std.os.windows.HWND;
const OVERLAPPED = std.os.windows.OVERLAPPED;
const BYTE = std.os.windows.BYTE;
const WORD = std.os.windows.WORD;
const LPCSTR = []const u8;
const ModemStat = enum(u32) {
    /// The CTS (clear-to-send) signal is on.
    MS_CTS_ON = 0x0010,
    /// The DSR (data-set-ready) signal is on.
    MS_DSR_ON = 0x0020,
    /// The ring indicator signal is on.
    MS_RING_ON = 0x0040,
    /// The RLSD (receive-line-signal-detect) signal is on.
    MS_RLSD_ON = 0x0080,
};
const someMask = enum(u32) {
    /// The hardware detected a break condition.
    CE_BREAK = 0x0010,
    /// The hardware detected a framing error.
    CE_FRAME = 0x0008,
    /// A character-buffer-overrun has occurred. The next character is lost.
    CE_OVERRUN = 0x0002,
    /// An input buffer overflow has occurred. There is either no room in the input buffer, or a character was received after the end-of-line (EOF) character.
    CE_RXOVER = 0x0001,
    /// The hardware detected a parity error.
    CE_RXPARITY = 0x0004,
};

/// Fills a specified DCB structure with values specified in a device-control string.
extern "kernel32" fn BuildCommDCBA(in_lpDef: *CSTR, out_lpDCB: *DCB) callconv(WINAPI) BOOL;

/// Translates a device-definition string into appropriate device-control block codes and places them into a device control block.
extern "kernel32" fn BuildCommDCBAndTimeoutsA(in_lpDef: *CSTR, out_lpDCB: *DCB, out_lpCommTimeouts: *COMMTIMEOUTS) callconv(WINAPI) BOOL;

/// Restores character transmission for a specified communications device and places the transmission line in a nonbreak state.
extern "kernel32" fn ClearCommBreak(in_hFile: HANDLE) callconv(WINAPI) BOOL;

/// Retrieves information about a communications error and reports the current status of a communications device.
extern "kernel32" fn ClearCommError(in_hfile: HANDLE, out_lpErrors: ?*DWORD, out_lpStat: ?*COMSTAT) BOOL;

/// Displays a driver-supplied configuration dialog box.
extern "kernel32" fn CommConfigDialogA(in_lpszName: *CSTR, in_hWnd: HWND, in_out_lpCC: *COMMCONFIG) callconv(WINAPI) BOOL;

/// Directs a specified communications device to perform an extended function.
extern "kernel32" fn EscapeCommFunction(in_hFile: HANDLE, in_dwFunc: DWORD) callconv(WINAPI) BOOL;

/// Retrieves the current configuration of a communications device.
extern "kernel32" fn GetCommConfig(in_hCommDev: HANDLE, out_lpCC: *COMMCONFIG, in_out_lpdwSize: *DWORD) callconv(WINAPI) BOOL;

/// Retrieves the value of the event mask for a specified communications device.
extern "kernel32" fn GetCommMask(in_hFile: HANDLE, out_lpEvtMask: *DWORD) callconv(WINAPI) BOOL;

/// Retrieves modem control-register values.
extern "kernel32" fn GetCommModemStatus(in_hFile: HANDLE, out_lpModemStat: *DWORD) callconv(WINAPI) BOOL;

/// Retrieves information about the communications properties for a specified communications device.
extern "kernel32" fn GetCommProperties(in_hFile: HANDLE, out_lpCommProp: *COMMPROP) callconv(WINAPI) BOOL;

/// Retrieves the current control settings for a specified communications device.
extern "kernel32" fn GetCommState(in_hFile: HANDLE, in_out_lpDCB: *DCB) callconv(WINAPI) BOOL;

/// Retrieves the time-out parameters for all read and write operations on a specified communications device.
extern "kernel32" fn GetCommTimeouts(in_hFile: HANDLE, out_lpCommTimeouts: *COMMTIMEOUTS) callconv(WINAPI) BOOL;

/// Retrieves the default configuration for the specified communications device.
extern "kernel32" fn GetDefaultCommConfigA(in_lpszName: *CSTR, out_lpCC: *COMMCONFIG, in_out_lpdwSize: *DWORD) callconv(WINAPI) BOOL;

/// Discards all characters from the output or input buffer of a specified communications resource.
extern "kernel32" fn PurgeComm(in_hFile: HANDLE, in_dwFlags: DWORD) callconv(WINAPI) BOOL;

/// Suspends character transmission for a specified communications device and places the transmission line in a break state.
extern "kernel32" fn SetCommBreak(in_hFile: HANDLE) callconv(WINAPI) BOOL;

/// Sets the current configuration of a communications device.
extern "kernel32" fn SetCommConfig(in_hCommDev: HANDLE, in_lpCC: *COMMCONFIG, in_dwSize: DWORD) callconv(WINAPI) BOOL;

/// Specifies a set of events to be monitored for a communications device.
extern "kernel32" fn SetCommMask(in_hFile: HANDLE, in_dwEvtMask: DWORD) callconv(WINAPI) BOOL;

/// Configures a communications device according to the specifications in a device-control block.
extern "kernel32" fn SetCommState(in_hFile: HANDLE, in_lpDCB: *DCB) callconv(WINAPI) BOOL;

/// Sets the time-out parameters for all read and write operations on a specified communications device.
extern "kernel32" fn SetCommTimeouts(in_hFile: HANDLE, in_lpCommTimeouts: *COMMTIMEOUTS) callconv(WINAPI) BOOL;

/// Sets the default configuration for a communications device.
extern "kernel32" fn SetDefaultCommConfigA(in_lpszName: *CSTR, in_lpCC: *COMMCONFIG, in_dwSize: DWORD) callconv(WINAPI) BOOL;

/// Initializes the communications parameters for a specified communications device.
extern "kernel32" fn SetupComm(in_hFile: HANDLE, in_dwInQueue: DWORD, in_dwOutQueue: DWORD) callconv(WINAPI) BOOL;

/// Transmits a specified character ahead of any pending data in the output buffer of the specified communications device.
extern "kernel32" fn TransmitCommChar(in_hFile: HANDLE, in_cChar: c_char) callconv(WINAPI) BOOL;

/// Waits for an event to occur for a specified communications device.
extern "kernel32" fn WaitCommEvent(in_hFile: HANDLE, out_lpEvntMask: *DWORD, in_lpOverlapped: *OVERLAPPED) callconv(WINAPI) BOOL;
