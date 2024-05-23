const std = @import("std");
const CSTR = []const u8;
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
const GUID = std.os.windows.GUID;
const PCWSTR = std.os.windows.PCWSTR;
const HDEVINFO = std.os.windows.HANDLE;
const DEVINST = std.os.windows.DWORD;
const ULONG_PTR = std.os.windows.ULONG_PTR;
const HKEY = std.os.windows.HKEY;
const CHAR = std.os.windows.CHAR;
const REGSAM = std.os.windows.REGSAM;
const LSTATUS = std.os.windows.LSTATUS;
const LPSTR = std.os.windows.LPSTR;
const WCHAR = std.os.windows.WCHAR;
const LPCSTR = std.os.windows.LPCSTR;

pub const SP_DEVINFO_DATA = extern struct {
    cbSize: DWORD,
    classGuid: GUID,
    devInst: DWORD,
    reserved: ULONG_PTR,
};

pub const COMMCONFIG = extern struct {
    dwSize: DWORD,
    wVersion: WORD,
    wReserved: WORD,
    dcb: DCB,
    dwProviderSubType: DWORD,
    dwProviderOffset: DWORD,
    dwProviderSize: DWORD,
    wcProviderData: [1]WCHAR,
};

pub const COMMPROP = extern struct {
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
};

pub const COMMTIMEOUTS = extern struct {
    ReadIntervalTimeout: DWORD,
    ReadTotalTimeoutMultiplier: DWORD,
    ReadTotalTimeoutConstant: DWORD,
    WriteTotalTimeoutMultiplier: DWORD,
    WriteTotalTimeoutConstant: DWORD,
};

pub const DCB = extern struct {
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
};

pub const COMSTAT = extern struct {
    const Self = @This();
    flags: u32,
    cbInQue: DWORD,
    cbOutQue: DWORD,

    pub fn ctsHold(self: *Self) bool {
        return self.flags & 1 != 0;
    }
    pub fn dsrHold(self: *Self) bool {
        return (self.flags >> 1) & 1 != 0;
    }
    pub fn rlsdHold(self: *Self) bool {
        return (self.flags >> 2) & 1 != 0;
    }
    pub fn xOffHold(self: *Self) bool {
        return (self.flags >> 3) & 1 != 0;
    }
    pub fn xOffSent(self: *Self) bool {
        return (self.flags >> 4) & 1 != 0;
    }
    pub fn eof(self: *Self) bool {
        return (self.flags >> 5) & 1 != 0;
    }
    pub fn txim(self: *Self) bool {
        return (self.flags >> 6) & 1 != 0;
    }
};

/// Fills a specified DCB structure with values specified in a device-control string.
pub extern "kernel32" fn BuildCommDCBA(
    in_lpDef: *CSTR,
    out_lpDCB: *DCB,
) callconv(WINAPI) BOOL;

/// Translates a device-definition string into appropriate device-control block codes and places them into a device control block.
pub extern "kernel32" fn BuildCommDCBAndTimeoutsA(
    in_lpDef: *CSTR,
    out_lpDCB: *DCB,
    out_lpCommTimeouts: *COMMTIMEOUTS,
) callconv(WINAPI) BOOL;

/// Restores character transmission for a specified communications device and places the transmission line in a nonbreak state.
pub extern "kernel32" fn ClearCommBreak(
    in_hFile: HANDLE,
) callconv(WINAPI) BOOL;

/// Retrieves information about a communications error and reports the current status of a communications device.
pub extern "kernel32" fn ClearCommError(
    in_hfile: HANDLE,
    out_lpErrors: ?*DWORD,
    out_lpStat: ?*COMSTAT,
) BOOL;

/// Displays a driver-supplied configuration dialog box.
pub extern "kernel32" fn CommConfigDialogA(
    in_lpszName: *CSTR,
    in_hWnd: HWND,
    in_out_lpCC: *COMMCONFIG,
) callconv(WINAPI) BOOL;

/// Closes an open object handle.
pub extern "kernel32" fn CloseHandle(
    in_hObject: HANDLE,
) callconv(WINAPI) BOOL;

/// Directs a specified communications device to perform an extended function.
pub extern "kernel32" fn EscapeCommFunction(
    in_hFile: HANDLE,
    in_dwFunc: DWORD,
) callconv(WINAPI) BOOL;

/// Retrieves the current configuration of a communications device.
pub extern "kernel32" fn GetCommConfig(
    in_hCommDev: HANDLE,
    out_lpCC: *COMMCONFIG,
    in_out_lpdwSize: *DWORD,
) callconv(WINAPI) BOOL;

/// Retrieves the value of the event mask for a specified communications device.
pub extern "kernel32" fn GetCommMask(
    in_hFile: HANDLE,
    out_lpEvtMask: *DWORD,
) callconv(WINAPI) BOOL;

/// Retrieves modem control-register values.
pub extern "kernel32" fn GetCommModemStatus(
    in_hFile: HANDLE,
    out_lpModemStat: *DWORD,
) callconv(WINAPI) BOOL;

/// Gets an array that contains the well-formed COM ports.
pub extern "kernel32" fn GetCommPorts(
    /// An array for the port numbers.
    out_lpPortNumbers: *ULONG,
    /// The length of the array in the lpPortNumbersparameter.
    in_uPortNumbersCount: ULONG,
    /// The number of port numbers written to the lpPortNumbers of the length of the array required for the port numbers.
    out_puPortNumbersFound: *ULONG,
) callconv(WINAPI) ULONG;

/// Retrieves information about the communications properties for a specified communications device.
pub extern "kernel32" fn GetCommProperties(
    in_hFile: HANDLE,
    out_lpCommProp: *COMMPROP,
) callconv(WINAPI) BOOL;

/// Retrieves the current control settings for a specified communications device.
pub extern "kernel32" fn GetCommState(
    in_hFile: HANDLE,
    in_out_lpDCB: *DCB,
) callconv(WINAPI) BOOL;

/// Retrieves the time-out parameters for all read and write operations on a specified communications device.
pub extern "kernel32" fn GetCommTimeouts(
    in_hFile: HANDLE,
    out_lpCommTimeouts: *COMMTIMEOUTS,
) callconv(WINAPI) BOOL;

/// Retrieves the default configuration for the specified communications device.
pub extern "kernel32" fn GetDefaultCommConfigA(
    in_lpszName: *CSTR,
    out_lpCC: *COMMCONFIG,
    in_out_lpdwSize: *DWORD,
) callconv(WINAPI) BOOL;

/// Attempts to open a communication device.
pub extern "kernel32" fn OpenCommPort(
    in_uPortNumber: ULONG,
    in_dwDesiredAccess: DWORD,
    in_dwFlagsAndAttributes: DWORD,
) callconv(WINAPI) HANDLE;

/// Discards all characters from the output or input buffer of a specified communications resource.
pub extern "kernel32" fn PurgeComm(
    in_hFile: HANDLE,
    in_dwFlags: DWORD,
) callconv(WINAPI) BOOL;

/// Suspends character transmission for a specified communications device and places the transmission line in a break state.
pub extern "kernel32" fn SetCommBreak(
    in_hFile: HANDLE,
) callconv(WINAPI) BOOL;

/// Sets the current configuration of a communications device.
pub extern "kernel32" fn SetCommConfig(
    in_hCommDev: HANDLE,
    in_lpCC: *COMMCONFIG,
    in_dwSize: DWORD,
) callconv(WINAPI) BOOL;

/// Specifies a set of events to be monitored for a communications device.
pub extern "kernel32" fn SetCommMask(
    in_hFile: HANDLE,
    in_dwEvtMask: DWORD,
) callconv(WINAPI) BOOL;

/// Configures a communications device according to the specifications in a device-control block.
pub extern "kernel32" fn SetCommState(
    in_hFile: HANDLE,
    in_lpDCB: *DCB,
) callconv(WINAPI) BOOL;

/// Sets the time-out parameters for all read and write operations on a specified communications device.
pub extern "kernel32" fn SetCommTimeouts(
    in_hFile: HANDLE,
    in_lpCommTimeouts: *COMMTIMEOUTS,
) callconv(WINAPI) BOOL;

/// Sets the default configuration for a communications device.
pub extern "kernel32" fn SetDefaultCommConfigA(
    in_lpszName: *CSTR,
    in_lpCC: *COMMCONFIG,
    in_dwSize: DWORD,
) callconv(WINAPI) BOOL;

/// Initializes the communications parameters for a specified communications device.
pub extern "kernel32" fn SetupComm(
    in_hFile: HANDLE,
    in_dwInQueue: DWORD,
    in_dwOutQueue: DWORD,
) callconv(WINAPI) BOOL;

/// Transmits a specified character ahead of any pending data in the output buffer of the specified communications device.
pub extern "kernel32" fn TransmitCommChar(
    in_hFile: HANDLE,
    in_cChar: c_char,
) callconv(WINAPI) BOOL;

/// Waits for an event to occur for a specified communications device.
pub extern "kernel32" fn WaitCommEvent(
    in_hFile: HANDLE,
    out_lpEvntMask: *DWORD,
    in_lpOverlapped: *OVERLAPPED,
) callconv(WINAPI) BOOL;

pub extern "setupapi" fn SetupDiGetClassDevsW(
    classGuid: ?*const GUID,
    enumerator: ?PCWSTR,
    hwndParanet: ?HWND,
    flags: DWORD,
) callconv(WINAPI) HDEVINFO;

pub extern "setupapi" fn SetupDiDestroyDeviceInfoList(
    device_info_set: HDEVINFO,
) callconv(WINAPI) BOOL;

pub extern "setupapi" fn SetupDiEnumDeviceInfo(
    devInfoSet: HDEVINFO,
    memberIndex: DWORD,
    device_info_data: *SP_DEVINFO_DATA,
) callconv(WINAPI) BOOL;

pub extern "setupapi" fn SetupDiGetDeviceInstanceIdA(
    device_info_set: HDEVINFO,
    device_info_data: *SP_DEVINFO_DATA,
    deviceInstanceId: *?CHAR,
    deviceInstanceIdSize: DWORD,
    requiredSize: ?*DWORD,
) callconv(WINAPI) BOOL;

pub extern "setupapi" fn SetupDiOpenDevRegKey(
    device_info_set: HDEVINFO,
    device_info_data: *SP_DEVINFO_DATA,
    scope: DWORD,
    hwProfile: DWORD,
    keyType: DWORD,
    samDesired: REGSAM,
) callconv(WINAPI) HKEY;

pub extern "advapi32" fn RegQueryValueExA(
    hKey: HKEY,
    lpValueName: LPSTR,
    lpReserved: ?*DWORD,
    lpType: ?*DWORD,
    lpData: ?[*]BYTE,
    lpcbData: ?*DWORD,
) callconv(WINAPI) LSTATUS;

pub extern "setupapi" fn SetupDiGetDeviceRegistryPropertyA(
    hDevInfo: HDEVINFO,
    pSpDevInfoData: *SP_DEVINFO_DATA,
    property: DWORD,
    propertyRegDataType: ?*DWORD,
    propertyBuffer: ?[*]BYTE,
    propertyBufferSize: DWORD,
    requiredSize: ?*DWORD,
) callconv(WINAPI) BOOL;

pub extern "cfgmgr32" fn CM_Get_Parent(
    pdnDevInst: *DEVINST,
    dnDevInst: DEVINST,
    ulFlags: ULONG,
) callconv(WINAPI) DWORD;

pub extern "cfgmgr32" fn CM_Get_Device_IDA(
    dnDevInst: DEVINST,
    buffer: LPSTR,
    bufferLen: ULONG,
    ulFlags: ULONG,
) callconv(WINAPI) DWORD;

extern "advapi32" fn RegOpenKeyExA(
    key: HKEY,
    lpSubKey: LPCSTR,
    ulOptions: DWORD,
    samDesired: REGSAM,
    phkResult: *HKEY,
) callconv(WINAPI) LSTATUS;

extern "advapi32" fn RegCloseKey(
    key: HKEY,
) callconv(WINAPI) LSTATUS;

extern "advapi32" fn RegEnumValueA(
    hKey: HKEY,
    dwIndex: DWORD,
    lpValueName: LPSTR,
    lpcchValueName: *DWORD,
    lpReserved: ?*DWORD,
    lpType: ?*DWORD,
    lpData: [*]BYTE,
    lpcbData: *DWORD,
) callconv(WINAPI) LSTATUS;