const std = @import("std");
const DCB = @import("DCB.zig");
const CSTR = []const u8;
const COMSTAT = @import("COMSTAT.zig");
const COMMPROP = @import("COMMPROP.zig");
const COMMCONFIG = @import("COMMCONFIG.zig");
const COMMTIMEOUTS = @import("COMMTIMEOUTS.zig");
const HANDLE = std.os.windows.HANDLE;
const DWORD = std.os.windows.DWORD;
const BOOL = std.os.windows.BOOL;
const HWND = std.os.windows.HWND;
const OVERLAPPED = std.os.windows.OVERLAPPED;
const BYTE = std.os.windows.BYTE;
const WORD = std.os.windows.WORD;
const ULONG = std.os.windows.ULONG;
const kernal32 = std.os.windows.kernel32;
const Win32Error = std.os.windows.Win32Error;
const WINAPI = std.os.windows.WINAPI;

/// Fills a specified DCB structure with values specified in a device-control string.
extern "kernel32" fn BuildCommDCBA(
    in_lpDef: *CSTR,
    out_lpDCB: *DCB,
) callconv(WINAPI) BOOL;

/// Translates a device-definition string into appropriate device-control block codes and places them into a device control block.
extern "kernel32" fn BuildCommDCBAndTimeoutsA(
    in_lpDef: *CSTR,
    out_lpDCB: *DCB,
    out_lpCommTimeouts: *COMMTIMEOUTS,
) callconv(WINAPI) BOOL;

/// Restores character transmission for a specified communications device and places the transmission line in a nonbreak state.
extern "kernel32" fn ClearCommBreak(
    in_hFile: HANDLE,
) callconv(WINAPI) BOOL;

/// Retrieves information about a communications error and reports the current status of a communications device.
extern "kernel32" fn ClearCommError(
    in_hfile: HANDLE,
    out_lpErrors: ?*DWORD,
    out_lpStat: ?*COMSTAT,
) BOOL;

/// Displays a driver-supplied configuration dialog box.
extern "kernel32" fn CommConfigDialogA(
    in_lpszName: *CSTR,
    in_hWnd: HWND,
    in_out_lpCC: *COMMCONFIG,
) callconv(WINAPI) BOOL;

/// Closes an open object handle.
extern "kernel32" fn CloseHandle(
    in_hObject: HANDLE,
) callconv(WINAPI) BOOL;

/// Directs a specified communications device to perform an extended function.
extern "kernel32" fn EscapeCommFunction(
    in_hFile: HANDLE,
    in_dwFunc: DWORD,
) callconv(WINAPI) BOOL;

/// Retrieves the current configuration of a communications device.
extern "kernel32" fn GetCommConfig(
    in_hCommDev: HANDLE,
    out_lpCC: *COMMCONFIG,
    in_out_lpdwSize: *DWORD,
) callconv(WINAPI) BOOL;

/// Retrieves the value of the event mask for a specified communications device.
extern "kernel32" fn GetCommMask(
    in_hFile: HANDLE,
    out_lpEvtMask: *DWORD,
) callconv(WINAPI) BOOL;

/// Retrieves modem control-register values.
extern "kernel32" fn GetCommModemStatus(
    in_hFile: HANDLE,
    out_lpModemStat: *DWORD,
) callconv(WINAPI) BOOL;

/// Gets an array that contains the well-formed COM ports.
extern "kernel32" fn GetCommPorts(
    /// An array for the port numbers.
    out_lpPortNumbers: *ULONG,
    /// The length of the array in the lpPortNumbersparameter.
    in_uPortNumbersCount: ULONG,
    /// The number of port numbers written to the lpPortNumbers of the length of the array required for the port numbers.
    out_puPortNumbersFound: *ULONG,
) callconv(WINAPI) ULONG;

/// Retrieves information about the communications properties for a specified communications device.
extern "kernel32" fn GetCommProperties(
    in_hFile: HANDLE,
    out_lpCommProp: *COMMPROP,
) callconv(WINAPI) BOOL;

/// Retrieves the current control settings for a specified communications device.
extern "kernel32" fn GetCommState(
    in_hFile: HANDLE,
    in_out_lpDCB: *DCB,
) callconv(WINAPI) std.os.windows.BOOL;

/// Retrieves the time-out parameters for all read and write operations on a specified communications device.
extern "kernel32" fn GetCommTimeouts(
    in_hFile: HANDLE,
    out_lpCommTimeouts: *COMMTIMEOUTS,
) callconv(WINAPI) BOOL;

/// Retrieves the default configuration for the specified communications device.
extern "kernel32" fn GetDefaultCommConfigA(
    in_lpszName: *CSTR,
    out_lpCC: *COMMCONFIG,
    in_out_lpdwSize: *DWORD,
) callconv(WINAPI) BOOL;

/// Attempts to open a communication device.
extern "kernel32" fn OpenCommPort(
    in_uPortNumber: ULONG,
    in_dwDesiredAccess: DWORD,
    in_dwFlagsAndAttributes: DWORD,
) callconv(WINAPI) HANDLE;

/// Discards all characters from the output or input buffer of a specified communications resource.
extern "kernel32" fn PurgeComm(
    in_hFile: HANDLE,
    in_dwFlags: DWORD,
) callconv(WINAPI) BOOL;

/// Suspends character transmission for a specified communications device and places the transmission line in a break state.
extern "kernel32" fn SetCommBreak(
    in_hFile: HANDLE,
) callconv(WINAPI) BOOL;

/// Sets the current configuration of a communications device.
extern "kernel32" fn SetCommConfig(
    in_hCommDev: HANDLE,
    in_lpCC: *COMMCONFIG,
    in_dwSize: DWORD,
) callconv(WINAPI) BOOL;

/// Specifies a set of events to be monitored for a communications device.
extern "kernel32" fn SetCommMask(
    in_hFile: HANDLE,
    in_dwEvtMask: DWORD,
) callconv(WINAPI) BOOL;

/// Configures a communications device according to the specifications in a device-control block.
extern "kernel32" fn SetCommState(
    in_hFile: HANDLE,
    in_lpDCB: *DCB,
) callconv(WINAPI) BOOL;

/// Sets the time-out parameters for all read and write operations on a specified communications device.
extern "kernel32" fn SetCommTimeouts(
    in_hFile: HANDLE,
    in_lpCommTimeouts: *COMMTIMEOUTS,
) callconv(WINAPI) BOOL;

/// Sets the default configuration for a communications device.
extern "kernel32" fn SetDefaultCommConfigA(
    in_lpszName: *CSTR,
    in_lpCC: *COMMCONFIG,
    in_dwSize: DWORD,
) callconv(WINAPI) BOOL;

/// Initializes the communications parameters for a specified communications device.
extern "kernel32" fn SetupComm(
    in_hFile: HANDLE,
    in_dwInQueue: DWORD,
    in_dwOutQueue: DWORD,
) callconv(WINAPI) BOOL;

/// Transmits a specified character ahead of any pending data in the output buffer of the specified communications device.
extern "kernel32" fn TransmitCommChar(
    in_hFile: HANDLE,
    in_cChar: c_char,
) callconv(WINAPI) BOOL;

/// Waits for an event to occur for a specified communications device.
extern "kernel32" fn WaitCommEvent(
    in_hFile: HANDLE,
    out_lpEvntMask: *DWORD,
    in_lpOverlapped: *OVERLAPPED,
) callconv(WINAPI) BOOL;
