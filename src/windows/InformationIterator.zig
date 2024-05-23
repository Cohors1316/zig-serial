index: DWORD,
device_info_set: HDEVINFO,

port_buffer: [256:0]u8,
sys_buffer: [256:0]u8,
name_buffer: [256:0]u8,
desc_buffer: [256:0]u8,
man_buffer: [256:0]u8,
serial_buffer: [256:0]u8,
hw_id: [256:0]u8,

pub fn init() !InfoIterator {
    var self: InfoIterator = undefined;
    self.index = 0;

    inline for (device_setup_tokens) |token| {
        const guid = token[0];
        const flags = token[1];

        self.device_info_set = externs.SetupDiGetClassDevsW(
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

pub fn deinit(self: *InfoIterator) void {
    _ = externs.SetupDiDestroyDeviceInfoList(self.device_info_set);
    self.* = undefined;
}

pub fn next(self: *InfoIterator) !?Information {
    var device_info_data: SP_DEVINFO_DATA = .{
        .cbSize = @sizeOf(SP_DEVINFO_DATA),
        .classGuid = std.mem.zeroes(GUID),
        .devInst = 0,
        .reserved = 0,
    };

    if (externs.SetupDiEnumDeviceInfo(self.device_info_set, self.index, &device_info_data) != std.os.windows.TRUE) {
        return null;
    }

    defer self.index += 1;

    var info: Information = std.mem.zeroes(Information);
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

    if (externs.SetupDiGetDeviceInstanceIdA(
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

const InfoIterator = @This();
const std = @import("std");
const HDEVINFO = std.os.windows.HANDLE;
const HWND = std.os.windows.HANDLE;
const HKEY = std.os.windows.HKEY;
const DWORD = std.os.windows.DWORD;
const Information = @import("../serial.zig").Information;
const DEVINST = std.os.windows.DWORD;
const KEY_READ = std.os.windows.KEY_READ;
const advapi32 = std.os.windows.advapi32;
const LPSTR = std.os.windows.LPSTR;
const NAME_MAX = std.os.windows.NAME_MAX;
const MAX_PATH = std.os.windows.MAX_PATH;
const FALSE = std.os.windows.FALSE;
const kernel32 = std.os.windows.kernel32;
const GUID = std.os.windows.GUID;
const ULONG_PTR = std.os.windows.ULONG_PTR;
const PCWSTR = std.os.windows.PCWSTR;
const externs = @import("externs.zig");
const SP_DEVINFO_DATA = externs.SP_DEVINFO_DATA;

fn getPortName(device_info_set: *const HDEVINFO, device_info_data: *SP_DEVINFO_DATA, port_name: [*]u8) DWORD {
    const hkey: HKEY = externs.SetupDiOpenDevRegKey(
        device_info_set.*,
        device_info_data,
        0x00000001, // #define DICS_FLAG_GLOBAL
        0,
        0x00000001, // #define DIREG_DEV,
        KEY_READ,
    );

    defer {
        _ = advapi32.RegCloseKey(hkey);
    }

    inline for (.{ "PortName", "PortNumber" }) |key_token| {
        var port_length: DWORD = NAME_MAX;
        var data_type: DWORD = 0;

        const result = externs.RegQueryValueExA(
            hkey,
            @as(LPSTR, @ptrCast(@constCast(key_token))),
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
    var data_type: DWORD = 0;
    var bytes_required: DWORD = MAX_PATH;

    const result = externs.SetupDiGetDeviceRegistryPropertyA(
        device_info_set.*,
        device_info_data,
        @intFromEnum(property),
        &data_type,
        property_str,
        NAME_MAX,
        &bytes_required,
    );

    if (result == FALSE) {
        std.debug.print("GetLastError: {}\n", .{kernel32.GetLastError()});
        bytes_required = 0;
    }

    return bytes_required;
}

fn getParentSerialNumber(devinst: DEVINST, devid: []const u8, serial_number: [*]u8) !DWORD {
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

            if (externs.CM_Get_Parent(&parent_id, child_inst, 0) != 0) return error.WindowsError;
            if (externs.CM_Get_Device_IDA(parent_id, @ptrCast(&local_buffer), 256, 0) != 0) return error.WindowsError;
            defer child_inst = parent_id;

            if (!std.mem.containsAtLeast(u8, local_buffer[0..255], 1, vidpid_slice)) continue;

            const length = try parseSerialNumber(local_buffer[0..255], serial_number);
            if (length > 0) return length;
        }
    }

    return error.WindowsError;
}

fn parseSerialNumber(devid: []const u8, serial_number: [*]u8) !DWORD {
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
            return @as(DWORD, @truncate(segment.len));
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


const Property = enum(DWORD) {
    SPDRP_DEVICEDESC = 0x00000000,
    SPDRP_MFG = 0x0000000B,
    SPDRP_FRIENDLYNAME = 0x0000000C,
};

// GUID taken from <devguid.h>
const DIGCF_PRESENT = 0x00000002;
const DIGCF_DEVICEINTERFACE = 0x00000010;
const device_setup_tokens = .{
    .{
        GUID{
            .Data1 = 0x4d36e978,
            .Data2 = 0xe325,
            .Data3 = 0x11ce,
            .Data4 = .{ 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 },
        },
        DIGCF_PRESENT,
    },
    .{
        GUID{
            .Data1 = 0x4d36e96d,
            .Data2 = 0xe325,
            .Data3 = 0x11ce,
            .Data4 = .{ 0xbf, 0xc1, 0x08, 0x00, 0x2b, 0xe1, 0x03, 0x18 },
        },
        DIGCF_PRESENT,
    },
    .{
        GUID{
            .Data1 = 0x86e0d1e0,
            .Data2 = 0x8089,
            .Data3 = 0x11d0,
            .Data4 = .{ 0x9c, 0xe4, 0x08, 0x00, 0x3e, 0x30, 0x1f, 0x73 },
        },
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE,
    },
    .{
        GUID{
            .Data1 = 0x2c7089aa,
            .Data2 = 0x2e0e,
            .Data3 = 0x11d1,
            .Data4 = .{ 0xb1, 0x14, 0x00, 0xc0, 0x4f, 0xc2, 0xaa, 0xe4 },
        },
        DIGCF_PRESENT | DIGCF_DEVICEINTERFACE,
    },
};
