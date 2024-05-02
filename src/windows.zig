const std = @import("std");
const serial = @import("zig-serial.zig");
const SerialPort = @import("SerialPort.zig");
const SerialPortDescription = serial.SerialPortDescription;
const PortInformation = serial.PortInformation;
const Config = serial.Config;
const ControlPins = serial.ControlPins;
const Parity = SerialPort.Parity;

pub fn configureSerialPort(port: std.fs.File, config: Config) !void {
    var dcb = std.mem.zeroes(DCB);
    dcb.DCBlength = @sizeOf(DCB);

    if (GetCommState(port.handle, &dcb) == 0)
        return error.WindowsError;

    var flags = DCBFlags.fromNumeric(dcb.flags);

    // std.log.err("{s} {s}", .{ dcb, flags });

    dcb.BaudRate = config.baud_rate;

    flags.fBinary = 1;
    flags.fParity = @intFromBool(config.parity != .none);
    flags.fOutxCtsFlow = @intFromBool(config.handshake == .hardware);
    flags.fOutxDsrFlow = 0;
    flags.fDtrControl = 0;
    flags.fDsrSensitivity = 0;
    flags.fTXContinueOnXoff = 0;
    flags.fOutX = @intFromBool(config.handshake == .software);
    flags.fInX = @intFromBool(config.handshake == .software);
    flags.fErrorChar = 0;
    flags.fNull = 0;
    flags.fRtsControl = @intFromBool(config.handshake == .hardware);
    flags.fAbortOnError = 0;
    dcb.flags = flags.toNumeric();

    dcb.wReserved = 0;
    dcb.ByteSize = switch (config.word_size) {
        .five => @as(u8, 5),
        .six => @as(u8, 6),
        .seven => @as(u8, 7),
        .eight => @as(u8, 8),
    };
    dcb.Parity = switch (config.parity) {
        .none => @as(u2, 0),
        .odd => @as(u2, 1),
        .even => @as(u2, 2),
        .mark => @as(u2, 3),
        .space => @as(u2, 4),
    };
    dcb.StopBits = switch (config.stop_bits) {
        .one => @as(u2, 0),
        .two => @as(u2, 2),
    };
    dcb.XonChar = 0x11;
    dcb.XoffChar = 0x13;
    dcb.wReserved1 = 0;

    if (SetCommState(port.handle, &dcb) == 0)
        return error.WindowsError;
}

const Line = enum { in, out, both };
pub fn flushSerialPort(port: std.fs.File, lines: Line) !void {
    const PURGE_RXCLEAR = 0x0008;
    const PURGE_TXCLEAR = 0x0004;
    // const PURGE_RXABORT = 0x0002;
    // const PURGE_TXABORT = 0x0001;
    const flags = switch (lines) {
        .in => PURGE_RXCLEAR,
        .out => PURGE_TXCLEAR,
        .both => PURGE_TXCLEAR | PURGE_RXCLEAR,
    };
    if (PurgeComm(port.handle, flags) == 0)
        return error.FlushError;
}

pub fn changeControlPins(port: std.fs.File, pins: ControlPins) !void {
    const CLRDTR = 6;
    const CLRRTS = 4;
    const SETDTR = 5;
    const SETRTS = 3;

    if (pins.dtr) |dtr| {
        if (EscapeCommFunction(port.handle, if (dtr) SETDTR else CLRDTR) == 0)
            return error.WindowsError;
    }
    if (pins.rts) |rts| {
        if (EscapeCommFunction(port.handle, if (rts) SETRTS else CLRRTS) == 0)
            return error.WindowsError;
    }
}
const HKEY = std.os.windows.HKEY;
const HWND = std.os.windows.HANDLE;
const HDEVINFO = std.os.windows.HANDLE;
const DEVINST = std.os.windows.DWORD;
const SP_DEVINFO_DATA = extern struct {
    cbSize: std.os.windows.DWORD,
    classGuid: std.os.windows.GUID,
    devInst: std.os.windows.DWORD,
    reserved: std.os.windows.ULONG_PTR,
};

const DCB = extern struct {
    DCBlength: std.os.windows.DWORD,
    BaudRate: std.os.windows.DWORD,
    flags: u32,
    wReserved: std.os.windows.WORD,
    XonLim: std.os.windows.WORD,
    XoffLim: std.os.windows.WORD,
    ByteSize: std.os.windows.BYTE,
    Parity: std.os.windows.BYTE,
    StopBits: std.os.windows.BYTE,
    XonChar: u8,
    XoffChar: u8,
    ErrorChar: u8,
    EofChar: u8,
    EvtChar: u8,
    wReserved1: std.os.windows.WORD,
};

const PortIterator = struct {
    const Self = @This();

    key: HKEY,
    index: u32,

    name: [256:0]u8 = undefined,
    name_size: u32 = 256,

    data: [256]u8 = undefined,
    filepath_data: [256]u8 = undefined,
    data_size: u32 = 256,

    pub fn init() !Self {
        const HKEY_LOCAL_MACHINE = @as(HKEY, @ptrFromInt(0x80000002));
        const KEY_READ = 0x20019;

        var self: Self = undefined;
        self.index = 0;
        if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM\\", 0, KEY_READ, &self.key) != 0)
            return error.WindowsError;

        return self;
    }

    pub fn deinit(self: *Self) void {
        _ = RegCloseKey(self.key);
        self.* = undefined;
    }

    pub fn next(self: *Self) !?SerialPortDescription {
        defer self.index += 1;

        self.name_size = 256;
        self.data_size = 256;

        return switch (RegEnumValueA(self.key, self.index, &self.name, &self.name_size, null, null, &self.data, &self.data_size)) {
            0 => SerialPortDescription{
                .file_name = try std.fmt.bufPrint(&self.filepath_data, "\\\\.\\{s}", .{self.data[0 .. self.data_size - 1]}),
                .display_name = self.data[0 .. self.data_size - 1],
                .driver = self.name[0..self.name_size],
            },
            259 => null,
            else => error.WindowsError,
        };
    }
};

const InformationIterator = struct {
    const Self = @This();

    index: std.os.windows.DWORD,
    device_info_set: HDEVINFO,

    port_buffer: [256:0]u8,
    sys_buffer: [256:0]u8,
    name_buffer: [256:0]u8,
    desc_buffer: [256:0]u8,
    man_buffer: [256:0]u8,
    serial_buffer: [256:0]u8,
    hw_id: [256:0]u8,

    const Property = enum(std.os.windows.DWORD) {
        SPDRP_DEVICEDESC = 0x00000000,
        SPDRP_MFG = 0x0000000B,
        SPDRP_FRIENDLYNAME = 0x0000000C,
    };

    // GUID taken from <devguid.h>
    const DIGCF_PRESENT = 0x00000002;
    const DIGCF_DEVICEINTERFACE = 0x00000010;
    const device_setup_tokens = .{
        .{ std.os.windows.GUID{ .Data1 = 0x4d36e978, .Data2 = 0xe325, .Data3 = 0x11ce, .Data4 = .{ 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 } }, DIGCF_PRESENT },
        .{ std.os.windows.GUID{ .Data1 = 0x4d36e96d, .Data2 = 0xe325, .Data3 = 0x11ce, .Data4 = .{ 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 } }, DIGCF_PRESENT },
        .{ std.os.windows.GUID{ .Data1 = 0x86e0d1e0, .Data2 = 0x8089, .Data3 = 0x11d0, .Data4 = .{ 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73 } }, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE },
        .{ std.os.windows.GUID{ .Data1 = 0x2c7089aa, .Data2 = 0x2e0e, .Data3 = 0x11d1, .Data4 = .{ 0xb1, 0x14, 0x00, 0xc0, 0x4f, 0xc2, 0xaa, 0xe4 } }, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE },
    };

    pub fn init() !Self {
        var self: Self = undefined;
        self.index = 0;

        inline for (device_setup_tokens) |token| {
            const guid = token[0];
            const flags = token[1];

            self.device_info_set = SetupDiGetClassDevsW(
                &guid,
                null,
                null,
                flags,
            );

            if (self.device_info_set != std.os.windows.INVALID_HANDLE_VALUE) break;
        }

        if (self.device_info_set == std.os.windows.INVALID_HANDLE_VALUE) return error.WindowsError;

        return self;
    }

    pub fn deinit(self: *Self) void {
        _ = SetupDiDestroyDeviceInfoList(self.device_info_set);
        self.* = undefined;
    }

    pub fn next(self: *Self) !?PortInformation {
        var device_info_data: SP_DEVINFO_DATA = .{
            .cbSize = @sizeOf(SP_DEVINFO_DATA),
            .classGuid = std.mem.zeroes(std.os.windows.GUID),
            .devInst = 0,
            .reserved = 0,
        };

        if (SetupDiEnumDeviceInfo(self.device_info_set, self.index, &device_info_data) != std.os.windows.TRUE) {
            return null;
        }

        defer self.index += 1;

        var info: PortInformation = std.mem.zeroes(PortInformation);
        @memset(&self.hw_id, 0);

        // NOTE: have not handled if port startswith("LPT")
        var length = getPortName(&self.device_info_set, &device_info_data, &self.port_buffer);
        info.port_name = self.port_buffer[0..length];

        info.system_location = try std.fmt.bufPrint(&self.sys_buffer, "\\\\.\\{s}", .{info.port_name});

        length = deviceRegistryProperty(&self.device_info_set, &device_info_data, Property.SPDRP_FRIENDLYNAME, &self.name_buffer);
        info.friendly_name = self.name_buffer[0..length];

        length = deviceRegistryProperty(&self.device_info_set, &device_info_data, Property.SPDRP_DEVICEDESC, &self.desc_buffer);
        info.description = self.desc_buffer[0..length];

        length = deviceRegistryProperty(&self.device_info_set, &device_info_data, Property.SPDRP_MFG, &self.man_buffer);
        info.manufacturer = self.man_buffer[0..length];

        if (SetupDiGetDeviceInstanceIdA(
            self.device_info_set,
            &device_info_data,
            @ptrCast(&self.hw_id),
            255,
            null,
        ) == std.os.windows.TRUE) {
            length = @as(u32, @truncate(std.mem.indexOfSentinel(u8, 0, &self.hw_id)));
            info.hw_id = self.hw_id[0..length];

            length = parseSerialNumber(&self.hw_id, &self.serial_buffer) catch 0;
            if (length == 0) {
                length = getParentSerialNumber(device_info_data.devInst, &self.hw_id, &self.serial_buffer) catch 0;
            }
            info.serial_number = self.serial_buffer[0..length];
            info.vid = parseVendorId(&self.hw_id) catch 0;
            info.pid = parseProductId(&self.hw_id) catch 0;
        } else {
            return error.WindowsError;
        }

        return info;
    }

    fn getPortName(device_info_set: *const HDEVINFO, device_info_data: *SP_DEVINFO_DATA, port_name: [*]u8) std.os.windows.DWORD {
        const hkey: HKEY = SetupDiOpenDevRegKey(
            device_info_set.*,
            device_info_data,
            0x00000001, // #define DICS_FLAG_GLOBAL
            0,
            0x00000001, // #define DIREG_DEV,
            std.os.windows.KEY_READ,
        );

        defer {
            _ = std.os.windows.advapi32.RegCloseKey(hkey);
        }

        inline for (.{ "PortName", "PortNumber" }) |key_token| {
            var port_length: std.os.windows.DWORD = std.os.windows.NAME_MAX;
            var data_type: std.os.windows.DWORD = 0;

            const result = RegQueryValueExA(
                hkey,
                @as(std.os.windows.LPSTR, @ptrCast(@constCast(key_token))),
                null,
                &data_type,
                port_name,
                &port_length,
            );

            // if this is valid, return now
            if (result == 0 and port_length > 0) {
                return port_length;
            }
        }

        return 0;
    }

    fn deviceRegistryProperty(device_info_set: *const HDEVINFO, device_info_data: *SP_DEVINFO_DATA, property: Property, property_str: [*]u8) std.os.windows.DWORD {
        var data_type: std.os.windows.DWORD = 0;
        var bytes_required: std.os.windows.DWORD = std.os.windows.MAX_PATH;

        const result = SetupDiGetDeviceRegistryPropertyA(
            device_info_set.*,
            device_info_data,
            @intFromEnum(property),
            &data_type,
            property_str,
            std.os.windows.NAME_MAX,
            &bytes_required,
        );

        if (result == std.os.windows.FALSE) {
            std.debug.print("GetLastError: {}\n", .{std.os.windows.kernel32.GetLastError()});
            bytes_required = 0;
        }

        return bytes_required;
    }

    fn getParentSerialNumber(devinst: DEVINST, devid: []const u8, serial_number: [*]u8) !std.os.windows.DWORD {
        if (std.mem.startsWith(u8, devid, "FTDI")) {
            // Should not be called on "FTDI" so just return the serial number.
            return try parseSerialNumber(devid, serial_number);
        } else if (std.mem.startsWith(u8, devid, "USB")) {
            // taken from pyserial
            const max_usb_device_tree_traversal_depth = 5;
            const start_vidpid = std.mem.indexOf(u8, devid, "VID") orelse return error.WindowsError;
            const vidpid_slice = devid[start_vidpid .. start_vidpid + 17]; // "VIDxxxx&PIDxxxx"

            // keep looping over parent device to extract serial number if it contains the target VID and PID.
            var depth: u8 = 0;
            var child_inst: DEVINST = devinst;
            while (depth <= max_usb_device_tree_traversal_depth) : (depth += 1) {
                var parent_id: DEVINST = undefined;
                var local_buffer: [256:0]u8 = std.mem.zeroes([256:0]u8);

                if (CM_Get_Parent(&parent_id, child_inst, 0) != 0) return error.WindowsError;
                if (CM_Get_Device_IDA(parent_id, @ptrCast(&local_buffer), 256, 0) != 0) return error.WindowsError;
                defer child_inst = parent_id;

                if (!std.mem.containsAtLeast(u8, local_buffer[0..255], 1, vidpid_slice)) continue;

                const length = try parseSerialNumber(local_buffer[0..255], serial_number);
                if (length > 0) return length;
            }
        }

        return error.WindowsError;
    }

    fn parseSerialNumber(devid: []const u8, serial_number: [*]u8) !std.os.windows.DWORD {
        var delimiter: ?[]const u8 = undefined;

        if (std.mem.startsWith(u8, devid, "USB")) {
            delimiter = "\\&";
        } else if (std.mem.startsWith(u8, devid, "FTDI")) {
            delimiter = "\\+";
        } else {
            // What to do here?
            delimiter = null;
        }

        if (delimiter) |del| {
            var it = std.mem.tokenize(u8, devid, del);

            // throw away the start
            _ = it.next();
            while (it.next()) |segment| {
                if (std.mem.startsWith(u8, segment, "VID_")) continue;
                if (std.mem.startsWith(u8, segment, "PID_")) continue;

                // If "MI_{d}{d}", this is an interface number. The serial number will have to be
                // sourced from the parent node. Probably do not have to check all these conditions.
                if (segment.len == 5 and std.mem.eql(u8, "MI_", segment[0..3]) and std.ascii.isDigit(segment[3]) and std.ascii.isDigit(segment[4])) return 0;

                @memcpy(serial_number, segment);
                return @as(std.os.windows.DWORD, @truncate(segment.len));
            }
        }

        return error.WindowsError;
    }

    fn parseVendorId(devid: []const u8) !u16 {
        var delimiter: ?[]const u8 = undefined;

        if (std.mem.startsWith(u8, devid, "USB")) {
            delimiter = "\\&";
        } else if (std.mem.startsWith(u8, devid, "FTDI")) {
            delimiter = "\\+";
        } else {
            delimiter = null;
        }

        if (delimiter) |del| {
            var it = std.mem.tokenize(u8, devid, del);

            while (it.next()) |segment| {
                if (std.mem.startsWith(u8, segment, "VID_")) {
                    return try std.fmt.parseInt(u16, segment[4..], 16);
                }
            }
        }

        return error.WindowsError;
    }

    fn parseProductId(devid: []const u8) !u16 {
        var delimiter: ?[]const u8 = undefined;

        if (std.mem.startsWith(u8, devid, "USB")) {
            delimiter = "\\&";
        } else if (std.mem.startsWith(u8, devid, "FTDI")) {
            delimiter = "\\+";
        } else {
            delimiter = null;
        }

        if (delimiter) |del| {
            var it = std.mem.tokenize(u8, devid, del);

            while (it.next()) |segment| {
                if (std.mem.startsWith(u8, segment, "PID_")) {
                    return try std.fmt.parseInt(u16, segment[4..], 16);
                }
            }
        }

        return error.WindowsError;
    }
};

const DCBFlags = struct {
    fBinary: u1, // u1
    fParity: u1, // u1
    fOutxCtsFlow: u1, // u1
    fOutxDsrFlow: u1, // u1
    fDtrControl: u2, // u2
    fDsrSensitivity: u1, // u1
    fTXContinueOnXoff: u1, // u1
    fOutX: u1, // u1
    fInX: u1, // u1
    fErrorChar: u1, // u1
    fNull: u1, // u1
    fRtsControl: u2, // u2
    fAbortOnError: u1, // u1
    fDummy2: u17 = 0, // u17

    // TODO: Packed structs please
    pub fn fromNumeric(value: u32) DCBFlags {
        var flags: DCBFlags = undefined;
        flags.fBinary = @as(u1, @truncate(value >> 0)); // u1
        flags.fParity = @as(u1, @truncate(value >> 1)); // u1
        flags.fOutxCtsFlow = @as(u1, @truncate(value >> 2)); // u1
        flags.fOutxDsrFlow = @as(u1, @truncate(value >> 3)); // u1
        flags.fDtrControl = @as(u2, @truncate(value >> 4)); // u2
        flags.fDsrSensitivity = @as(u1, @truncate(value >> 6)); // u1
        flags.fTXContinueOnXoff = @as(u1, @truncate(value >> 7)); // u1
        flags.fOutX = @as(u1, @truncate(value >> 8)); // u1
        flags.fInX = @as(u1, @truncate(value >> 9)); // u1
        flags.fErrorChar = @as(u1, @truncate(value >> 10)); // u1
        flags.fNull = @as(u1, @truncate(value >> 11)); // u1
        flags.fRtsControl = @as(u2, @truncate(value >> 12)); // u2
        flags.fAbortOnError = @as(u1, @truncate(value >> 14)); // u1
        flags.fDummy2 = @as(u17, @truncate(value >> 15)); // u17
        return flags;
    }

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

test "DCBFlags" {
    var rand: u32 = 0;
    try std.os.getrandom(@as(*[4]u8, @ptrCast(&rand)));
    var flags = DCBFlags.fromNumeric(rand);
    try std.testing.expectEqual(rand, flags.toNumeric());
}

extern "advapi32" fn RegOpenKeyExA(
    key: HKEY,
    lpSubKey: std.os.windows.LPCSTR,
    ulOptions: std.os.windows.DWORD,
    samDesired: std.os.windows.REGSAM,
    phkResult: *HKEY,
) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;
extern "advapi32" fn RegCloseKey(key: HKEY) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;
extern "advapi32" fn RegEnumValueA(
    hKey: HKEY,
    dwIndex: std.os.windows.DWORD,
    lpValueName: std.os.windows.LPSTR,
    lpcchValueName: *std.os.windows.DWORD,
    lpReserved: ?*std.os.windows.DWORD,
    lpType: ?*std.os.windows.DWORD,
    lpData: [*]std.os.windows.BYTE,
    lpcbData: *std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;
extern "advapi32" fn RegQueryValueExA(
    hKey: HKEY,
    lpValueName: std.os.windows.LPSTR,
    lpReserved: ?*std.os.windows.DWORD,
    lpType: ?*std.os.windows.DWORD,
    lpData: ?[*]std.os.windows.BYTE,
    lpcbData: ?*std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.LSTATUS;
extern "setupapi" fn SetupDiGetClassDevsW(
    classGuid: ?*const std.os.windows.GUID,
    enumerator: ?std.os.windows.PCWSTR,
    hwndParanet: ?HWND,
    flags: std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) HDEVINFO;
extern "setupapi" fn SetupDiEnumDeviceInfo(
    devInfoSet: HDEVINFO,
    memberIndex: std.os.windows.DWORD,
    device_info_data: *SP_DEVINFO_DATA,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "setupapi" fn SetupDiDestroyDeviceInfoList(device_info_set: HDEVINFO) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "setupapi" fn SetupDiOpenDevRegKey(
    device_info_set: HDEVINFO,
    device_info_data: *SP_DEVINFO_DATA,
    scope: std.os.windows.DWORD,
    hwProfile: std.os.windows.DWORD,
    keyType: std.os.windows.DWORD,
    samDesired: std.os.windows.REGSAM,
) callconv(std.os.windows.WINAPI) HKEY;
extern "setupapi" fn SetupDiGetDeviceRegistryPropertyA(
    hDevInfo: HDEVINFO,
    pSpDevInfoData: *SP_DEVINFO_DATA,
    property: std.os.windows.DWORD,
    propertyRegDataType: ?*std.os.windows.DWORD,
    propertyBuffer: ?[*]std.os.windows.BYTE,
    propertyBufferSize: std.os.windows.DWORD,
    requiredSize: ?*std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "setupapi" fn SetupDiGetDeviceInstanceIdA(
    device_info_set: HDEVINFO,
    device_info_data: *SP_DEVINFO_DATA,
    deviceInstanceId: *?std.os.windows.CHAR,
    deviceInstanceIdSize: std.os.windows.DWORD,
    requiredSize: ?*std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "cfgmgr32" fn CM_Get_Parent(
    pdnDevInst: *DEVINST,
    dnDevInst: DEVINST,
    ulFlags: std.os.windows.ULONG,
) callconv(std.os.windows.WINAPI) std.os.windows.DWORD;
extern "cfgmgr32" fn CM_Get_Device_IDA(
    dnDevInst: DEVINST,
    buffer: std.os.windows.LPSTR,
    bufferLen: std.os.windows.ULONG,
    ulFlags: std.os.windows.ULONG,
) callconv(std.os.windows.WINAPI) std.os.windows.DWORD;
extern "kernel32" fn GetCommState(
    hFile: std.os.windows.HANDLE,
    lpDCB: *DCB,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "kernel32" fn SetCommState(
    hFile: std.os.windows.HANDLE,
    lpDCB: *DCB,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "kernel32" fn PurgeComm(
    hFile: std.os.windows.HANDLE,
    dwFlags: std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
extern "kernel32" fn EscapeCommFunction(
    hFile: std.os.windows.HANDLE,
    dwFunc: std.os.windows.DWORD,
) callconv(std.os.windows.WINAPI) std.os.windows.BOOL;
