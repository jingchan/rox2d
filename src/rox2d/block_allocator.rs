


const BLOCK_SIZE_COUNT: usize = 14;
const CHUNK_SIZE: usize = 16 * 1024;
const MAX_BLOCK_SIZE: usize = 640;
const CHUNK_ARRAY_INCREMENT: usize = 128;

// These are the supported object sizes. Actual allocations are rounded up the next size.
const BLOCK_SIZES: [usize; BLOCK_SIZE_COUNT] =
[
	16,		// 0
	32,		// 1
	64,		// 2
	96,		// 3
	128,	// 4
	160,	// 5
	192,	// 6
	224,	// 7
	256,	// 8
	320,	// 9
	384,	// 10
	448,	// 11
	512,	// 12
	640,	// 13
];

// This maps an arbitrary allocation size to a suitable slot in b2_blockSizes.
struct SizeMap {
    values: [u8; MAX_BLOCK_SIZE + 1],
}

impl SizeMap {
    pub fn new() -> Self {
        let mut values = [0; MAX_BLOCK_SIZE + 1];
        let mut j = 0;
        values[0] = 0;
        for i in 1..=MAX_BLOCK_SIZE {
            assert!(j < BLOCK_SIZE_COUNT);
            if i <= BLOCK_SIZES[j] {
                values[i] = j as u8;
            } else {
                j += 1;
                values[i] = j as u8;
            }
        }
        Self { values }
    }
}

struct Chunk {
    block_size: usize,
    blocks: Vec<Block>,
}

struct Block {
    next: Option<Box<Block>>,
}

pub struct BlockAllocator {
    chunks: Vec<Chunk>,

    free_lists: [Vec<Block>; BLOCK_SIZE_COUNT],
}

impl BlockAllocator {
    pub fn new() -> Self {
        Self {
            chunks: Vec::new(),
            free_lists: [Vec::new(); BLOCK_SIZE_COUNT],
        }
    }

    pub fn allocate(&mut self, size: usize) -> *mut u8 {
        if size == 0 {
            return std::ptr::null_mut();
        }

        assert!(size <= MAX_BLOCK_SIZE);

        let index = SizeMap::new().values[size] as usize;
        assert!(index < BLOCK_SIZE_COUNT);

        if let Some(block) = self.free_lists[index].pop() {
            Box::into_raw(Box::new(block)) as *mut u8
        } else {
            self.chunks.push(Chunk {
                block_size: BLOCK_SIZES[index],
                blocks: Vec::new(),
            });
            let chunk = self.chunks.last_mut().unwrap();
            chunk.blocks.push(Block { next: None });
            Box::into_raw(Box::new(chunk.blocks.last_mut().unwrap())) as *mut u8
        }
    }
}



// 	else
// 	{
// 		if (m_chunkCount == m_chunkSpace)
// 		{
// 			b2Chunk* oldChunks = m_chunks;
// 			m_chunkSpace += b2_chunkArrayIncrement;
// 			m_chunks = (b2Chunk*)b2Alloc(m_chunkSpace * sizeof(b2Chunk));
// 			memcpy(m_chunks, oldChunks, m_chunkCount * sizeof(b2Chunk));
// 			memset(m_chunks + m_chunkCount, 0, b2_chunkArrayIncrement * sizeof(b2Chunk));
// 			b2Free(oldChunks);
// 		}

// 		b2Chunk* chunk = m_chunks + m_chunkCount;
// 		chunk->blocks = (b2Block*)b2Alloc(b2_chunkSize);
// #if defined(_DEBUG)
// 		memset(chunk->blocks, 0xcd, b2_chunkSize);
// #endif
// 		int32 blockSize = b2_blockSizes[index];
// 		chunk->blockSize = blockSize;
// 		int32 blockCount = b2_chunkSize / blockSize;
// 		b2Assert(blockCount * blockSize <= b2_chunkSize);
// 		for (int32 i = 0; i < blockCount - 1; ++i)
// 		{
// 			b2Block* block = (b2Block*)((int8*)chunk->blocks + blockSize * i);
// 			b2Block* next = (b2Block*)((int8*)chunk->blocks + blockSize * (i + 1));
// 			block->next = next;
// 		}
// 		b2Block* last = (b2Block*)((int8*)chunk->blocks + blockSize * (blockCount - 1));
// 		last->next = nullptr;

// 		m_freeLists[index] = chunk->blocks->next;
// 		++m_chunkCount;

// 		return chunk->blocks;
// 	}
// }

// void b2BlockAllocator::Free(void* p, int32 size)
// {
// 	if (size == 0)
// 	{
// 		return;
// 	}

// 	b2Assert(0 < size);

// 	if (size > b2_maxBlockSize)
// 	{
// 		b2Free(p);
// 		return;
// 	}

// 	int32 index = b2_sizeMap.values[size];
// 	b2Assert(0 <= index && index < b2_blockSizeCount);

// #if defined(_DEBUG)
// 	// Verify the memory address and size is valid.
// 	int32 blockSize = b2_blockSizes[index];
// 	bool found = false;
// 	for (int32 i = 0; i < m_chunkCount; ++i)
// 	{
// 		b2Chunk* chunk = m_chunks + i;
// 		if (chunk->blockSize != blockSize)
// 		{
// 			b2Assert(	(int8*)p + blockSize <= (int8*)chunk->blocks ||
// 						(int8*)chunk->blocks + b2_chunkSize <= (int8*)p);
// 		}
// 		else
// 		{
// 			if ((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + b2_chunkSize)
// 			{
// 				found = true;
// 			}
// 		}
// 	}

// 	b2Assert(found);

// 	memset(p, 0xfd, blockSize);
// #endif

// 	b2Block* block = (b2Block*)p;
// 	block->next = m_freeLists[index];
// 	m_freeLists[index] = block;
// }

// void b2BlockAllocator::Clear()
// {
// 	for (int32 i = 0; i < m_chunkCount; ++i)
// 	{
// 		b2Free(m_chunks[i].blocks);
// 	}

// 	m_chunkCount = 0;
// 	memset(m_chunks, 0, m_chunkSpace * sizeof(b2Chunk));
// 	memset(m_freeLists, 0, sizeof(m_freeLists));
// }
