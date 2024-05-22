flags: u32,
cbInQue: DWORD,
cbOutQue: DWORD,

pub fn ctsHold(self: *COMSTAT) bool {
    return self.flags & 1 != 0;
}

pub fn dsrHold(self: *COMSTAT) bool {
    return (self.flags >> 1) & 1 != 0;
}

pub fn rlsdHold(self: *COMSTAT) bool {
    return (self.flags >> 2) & 1 != 0;
}

pub fn xOffHold(self: *COMSTAT) bool {
    return (self.flags >> 3) & 1 != 0;
}

pub fn xOffSent(self: *COMSTAT) bool {
    return (self.flags >> 4) & 1 != 0;
}

pub fn eof(self: *COMSTAT) bool {
    return (self.flags >> 5) & 1 != 0;
}

pub fn txim(self: *COMSTAT) bool {
    return (self.flags >> 6) & 1 != 0;
}

const COMSTAT = @This();
const DWORD = @import("std").os.windows.DWORD;
