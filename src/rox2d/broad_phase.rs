use super::{collision::Aabb, dynamic_tree::DynamicTree, Vec2};

pub struct Pair {
    proxy_id_a: usize,
    proxy_id_b: usize,
}

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

    pub fn move_proxy(
        &mut self,
        proxy_id: usize,
        aabb: &Aabb,
        displacement: Vec2,
    ) {
        let buffer = self.tree.move_proxy(proxy_id, aabb, displacement);
        if buffer {
            self.move_buffer.push(proxy_id);
        }
    }
}

// void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
// {
// 	bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
// 	if (buffer)
// 	{
// 		BufferMove(proxyId);
// 	}
// }
