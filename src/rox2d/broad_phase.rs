use super::common::Vec2;
use super::{collision::Aabb, dynamic_tree::DynamicTree};

#[derive(Debug, Clone)]
pub struct Pair {
    proxy_id_a: usize,
    proxy_id_b: usize,
}

#[derive(Debug, Clone)]
pub struct BroadPhase {
    pub tree: DynamicTree,
    // proxy_count: usize,
    move_buffer: Vec<usize>,
    // move_capacity: usize,
    // move_count: usize,
    pair_buffer: Vec<Pair>,
    // pair_capacity: usize,
    // pair_count: usize,
    query_proxy_id: usize,
}

impl BroadPhase {
    pub fn new() -> Self {
        Self {
            tree: DynamicTree::new(),
            // proxy_count: 0,
            move_buffer: Vec::new(),
            // move_capacity: 0,
            // move_count: 0,
            pair_buffer: Vec::new(),
            // pair_capacity: 0,
            // pair_count: 0,
            query_proxy_id: 0,
        }
    }

    // pub fn create_proxy(&mut self, aabb: &Aabb, user_data: usize) -> usize {
    pub fn create_proxy(&mut self, aabb: &Aabb) -> usize {
        let proxy_id = self.tree.create_proxy(aabb);

        // Add to list of moved proxies.
        self.move_buffer.push(proxy_id);
        proxy_id
    }

    // int32 b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData)
    // {
    // 	int32 proxyId = m_tree.CreateProxy(aabb, userData);
    // 	++m_proxyCount;
    // 	BufferMove(proxyId);
    // 	return proxyId;
    // }
    // Called to let broadphase know that the proxy was moved.
    pub fn move_proxy(
        &mut self,
        proxy_id: usize,
        aabb: &Aabb,
        displacement: Vec2,
    ) {
        // Apply the move within the dynamic tree.
        let buffer = self.tree.move_proxy(proxy_id, aabb, displacement);

        // Add to list of moved proxies.
        if buffer {
            self.move_buffer.push(proxy_id);
        }
    }

    /// Update the pairs. This results in pair callbacks. This can only add pairs.
    pub fn update_pairs<F: FnOnce()>(&self, callback: F) {
        // Reset pair buffer
        self.pair_buffer.clear();

        // Perform tree queries for all moving proxies.
        for i in 0..self.move_buffer.len() {
            self.query_proxy_id = self.move_buffer[i];
            if self.query_proxy_id == usize::MAX {
                continue;
            }

            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            let fat_aabb = self.tree.get_fat_aabb(self.query_proxy_id);

            // Query tree, create pairs and add them pair buffer.
            self.tree.query(&fat_aabb, &mut |proxy_id| {
                // A proxy cannot form a pair with itself.
                if proxy_id == self.query_proxy_id {
                    return true;
                }

                // Grow the pair buffer as needed.
                // if self.pair_buffer.len() == self.pair_buffer.capacity() {
                // self.pair_buffer.reserve(2 * self.pair_buffer.len());
                // }

                // Add the pair.
                self.pair_buffer.push(Pair {
                    proxy_id_a: usize::min(self.query_proxy_id, proxy_id),
                    proxy_id_b: usize::max(self.query_proxy_id, proxy_id),
                });

                true
            });
        }

        // Reset move buffer
        self.move_buffer.clear();
    }
    // void b2BroadPhase::UpdatePairs(T* callback)
    // {
    // 	// Reset pair buffer
    // 	m_pairCount = 0;

    // 	// Perform tree queries for all moving proxies.
    // 	for (int32 i = 0; i < m_moveCount; ++i)
    // 	{
    // 		m_queryProxyId = m_moveBuffer[i];
    // 		if (m_queryProxyId == e_nullProxy)
    // 		{
    // 			continue;
    // 		}

    // 		// We have to query the tree with the fat AABB so that
    // 		// we don't fail to create a pair that may touch later.
    // 		const b2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);

    // 		// Query tree, create pairs and add them pair buffer.
    // 		m_tree.Query(this, fatAABB);
    // 	}

    // 	// Send pairs to caller
    // 	for (int32 i = 0; i < m_pairCount; ++i)
    // 	{
    // 		b2Pair* primaryPair = m_pairBuffer + i;
    // 		void* userDataA = m_tree.GetUserData(primaryPair->proxyIdA);
    // 		void* userDataB = m_tree.GetUserData(primaryPair->proxyIdB);

    // 		callback->AddPair(userDataA, userDataB);
    // 	}

    // 	// Clear move flags
    // 	for (int32 i = 0; i < m_moveCount; ++i)
    // 	{
    // 		int32 proxyId = m_moveBuffer[i];
    // 		if (proxyId == e_nullProxy)
    // 		{
    // 			continue;
    // 		}

    // 		m_tree.ClearMoved(proxyId);
    // 	}

    // 	// Reset move buffer
    // 	m_moveCount = 0;
    // }
}

// void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
// {
// 	bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
// 	if (buffer)
// 	{
// 		BufferMove(proxyId);
// 	}
// }
