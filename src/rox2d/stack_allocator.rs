const STACK_SIZE: usize = 100 * 1024; // 100k
const MAX_STACK_ENTRIES: usize = 32;

#[derive(Clone, Copy, Debug)]
struct StackEntry {
    data: *mut u8,
    size: i32,
    used_malloc: bool,
}

// This is a stack allocator used for fast per step allocations.
//
// You must nest allocate/free pairs. The code will assert if you try to
// interleave multiple allocate/free pairs.
#[derive(Debug)]
pub struct StackAllocator {
    data: [u8; STACK_SIZE],
    index: i32,
    allocation: i32,
    max_allocation: i32,
    entries: [StackEntry; MAX_STACK_ENTRIES],
    entry_count: i32,
}

impl StackAllocator {
    pub fn new() -> Self {
        Self {
            data: [0; STACK_SIZE],
            index: 0,
            allocation: 0,
            max_allocation: 0,
            entries: [StackEntry {
                data: std::ptr::null_mut(),
                size: 0,
                used_malloc: false,
            }; MAX_STACK_ENTRIES],
            entry_count: 0,
        }
    }

    pub fn allocate(&mut self, size: i32) -> *mut u8 {
        assert!(self.entry_count < MAX_STACK_ENTRIES as i32);

        let entry = &mut self.entries[self.entry_count as usize];
        entry.size = size;
        if self.index + size > STACK_SIZE as i32 {
            entry.data = unsafe { libc::malloc(size as usize) as *mut u8 };
            entry.used_malloc = true;
        } else {
            entry.data = &mut self.data[self.index as usize];
            entry.used_malloc = false;
            self.index += size;
        }

        self.allocation += size;
        self.max_allocation = self.max_allocation.max(self.allocation);
        self.entry_count += 1;

        entry.data
    }

    pub fn free(&mut self, p: *mut u8) {
        assert!(self.entry_count > 0);
        let entry = &mut self.entries[self.entry_count as usize - 1];
        assert!(p == entry.data);
        if entry.used_malloc {
            unsafe { libc::free(p as *mut libc::c_void) };
        } else {
            self.index -= entry.size;
        }
        self.allocation -= entry.size;
        self.entry_count -= 1;
    }

    pub fn get_max_allocation(&self) -> i32 {
        self.max_allocation
    }
}
