use super::{collision::Aabb, Vec2};

enum TreeNodeConnection {
    None,
    /// Tree mode.
    Parent(usize),
    /// Linked-list mode.
    Next(usize),
}

/// A node in the dynamic tree. The client does not interact with this directly.
pub struct TreeNode {
    aabb: Aabb,
    // user_data: Option<Box<dyn std::any::Any>>,
    connection: TreeNodeConnection,

    child1: Option<usize>,
    child2: Option<usize>,
    height: i32,

    moved: bool,
}

impl TreeNode {
    pub fn is_leaf(&self) -> bool {
        self.child1.is_none()
    }
}

/// A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
/// A dynamic tree arranges data in a binary tree to accelerate
/// queries such as volume queries and ray casts. Leafs are proxies
/// with an AABB. In the tree we expand the proxy AABB by b2_fatAABBFactor
/// so that the proxy AABB is bigger than the client object. This allows the client
/// object to move by small amounts without triggering a tree update.
///
/// Nodes are pooled and relocatable, so we use node indices rather than pointers.
pub struct DynamicTree {
    root: Option<usize>,
    nodes: Vec<TreeNode>,
    // node_count: usize,
    // node_capacity: usize,
    free_list: Vec<usize>,
    insertion_count: usize,
}

impl DynamicTree {
    pub fn new() -> Self {
        let mut nodes = Vec::with_capacity(16);
        for i in 0..nodes.capacity() - 1 {
            nodes.push(TreeNode {
                aabb: Aabb::default(),
                connection: TreeNodeConnection::Next(i + 1),
                child1: None,
                child2: None,
                height: -1,
                moved: false,
            });
        }
        nodes.push(TreeNode {
            aabb: Aabb::default(),
            connection: TreeNodeConnection::None,
            child1: None,
            child2: None,
            height: -1,
            moved: false,
        });

        Self {
            root: None,
            /// Preallocate capacity?
            nodes,
            // node_count: 0,
            // node_capacity: 0,
            free_list: Vec::new(),
            insertion_count: 0,
        }
    }

    // b2DynamicTree::b2DynamicTree()
    // {
    // 	m_root = b2_nullNode;

    // 	m_nodeCapacity = 16;
    // 	m_nodeCount = 0;
    // 	m_nodes = (b2TreeNode*)b2Alloc(m_nodeCapacity * sizeof(b2TreeNode));
    // 	memset(m_nodes, 0, m_nodeCapacity * sizeof(b2TreeNode));

    // 	// Build a linked list for the free list.
    // 	for (int32 i = 0; i < m_nodeCapacity - 1; ++i)
    // 	{
    // 		m_nodes[i].next = i + 1;
    // 		m_nodes[i].height = -1;
    // 	}
    // 	m_nodes[m_nodeCapacity-1].next = b2_nullNode;
    // 	m_nodes[m_nodeCapacity-1].height = -1;
    // 	m_freeList = 0;

    // 	m_insertionCount = 0;
    // }

    pub fn move_proxy(
        &mut self,
        proxy_id: usize,
        aabb: &Aabb,
        displacement: Vec2,
    ) -> bool {
        let node = &mut self.nodes[proxy_id];
        let old_aabb = node.aabb;
        node.aabb = *aabb;

        // Extend AABB
        let r = Vec2::new(0.1, 0.1);
        node.aabb.lower_bound -= r;
        node.aabb.upper_bound += r;

        // Predict AABB movement
        let d = Vec2::new(0.1, 0.1) * displacement;

        if d.x < 0.0 {
            node.aabb.lower_bound.x += d.x;
        } else {
            node.aabb.upper_bound.x += d.x;
        }

        if d.y < 0.0 {
            node.aabb.lower_bound.y += d.y;
        } else {
            node.aabb.upper_bound.y += d.y;
        }

        let buffer = self.insertion_count < 3;

        if node.aabb.contains(&old_aabb) {
            return buffer;
        }

        self.remove_leaf(proxy_id);

        self.insert_leaf(proxy_id);

        return true;
    }

    fn insert_leaf(&mut self, leaf: usize) {
        self.insertion_count += 1;

        if self.root.is_none() {
            self.root = Some(leaf);
            self.nodes[leaf].connection = TreeNodeConnection::None;
            return;
        }

        // Find the best sibling for this node
        let leaf_aabb = self.nodes[leaf].aabb;
        let mut index = self.root.unwrap();
        while self.nodes[index].child1.is_some() {
            let child1 = self.nodes[index].child1.unwrap();
            let child2 = self.nodes[index].child2.unwrap();

            // Note: Why is this being called area when it's the perimeter?
            let area = self.nodes[index].aabb.get_perimeter();

            let mut combined_aabb = Aabb::default();
            combined_aabb = self.nodes[index].aabb.combine(&leaf_aabb);
            let combined_area = combined_aabb.get_perimeter();

            // Cost of creating a new parent for this node and the new leaf
            let cost = 2.0 * combined_area;

            // Minimum cost of pushing the leaf further down the tree
            let inheritance_cost = 2.0 * (combined_area - area);

            // Cost of descending into child1
            let cost1;
            if self.nodes[child1].aabb.contains(&leaf_aabb) {
                cost1 = inheritance_cost;
            } else {
                let mut aabb = Aabb::default();
                aabb = self.nodes[child1].aabb.combine(&leaf_aabb);
                cost1 = aabb.get_perimeter() + inheritance_cost;
            }

            // Cost of descending into child2
            let cost2;
            if self.nodes[child2].aabb.contains(&leaf_aabb) {
                cost2 = inheritance_cost;
            } else {
                let mut aabb = Aabb::default();
                aabb = self.nodes[child2].aabb.combine(&leaf_aabb);
                cost2 = aabb.get_perimeter() + inheritance_cost;
            }

            // Descend according to the minimum cost.
            if cost < cost1 && cost < cost2 {
                break;
            }

            // Descend
            if cost1 < cost2 {
                index = child1;
            } else {
                index = child2;
            }
        }

        let sibling = index;

        // Create a new parent.
        let old_parent = self.nodes[sibling].connection;
        let new_parent = self.allocate_node();
        self.nodes[new_parent].connection = old_parent;
        self.nodes[new_parent].aabb =
            self.nodes[sibling].aabb.combine(&leaf_aabb);
        self.nodes[new_parent].height = self.nodes[sibling].height + 1;

        if let TreeNodeConnection::Parent(old_parent) = old_parent {
            // The sibling was not the root.
            if self.nodes[old_parent].child1 == Some(sibling) {
                self.nodes[old_parent].child1 = new_parent;
            } else {
                self.nodes[old_parent].child2 = new_parent;
            }

            self.nodes[new_parent].child1 = sibling;
            self.nodes[new_parent].child2 = leaf;
            self.nodes[sibling].connection =
                TreeNodeConnection::Parent(new_parent);
            self.nodes[leaf].connection =
                TreeNodeConnection::Parent(new_parent);
        } else {
            // The sibling was the root.
            self.nodes[new_parent].child1 = sibling;
            self.nodes[new_parent].child2 = leaf;
            self.nodes[sibling].connection =
                TreeNodeConnection::Parent(new_parent);
            self.nodes[leaf].connection =
                TreeNodeConnection::Parent(new_parent);
            self.root = Some(new_parent);
        }

        // Walk back up the tree fixing heights and AABBs
        let mut index = self.nodes[leaf].connection;
        while let TreeNodeConnection::Parent(index) = index {
            index = self.balance(index);

            let child1 = self.nodes[index].child1.unwrap();
            let child2 = self.nodes[index].child2.unwrap();

            self.nodes[index].height =
                1 + self.nodes[child1].height.max(self.nodes[child2].height);
            self.nodes[index].aabb =
                self.nodes[child1].aabb.combine(&self.nodes[child2].aabb);

            index = match self.nodes[index].connection {
                TreeNodeConnection::Parent(index) => index,
                _ => panic!("No parent found"),
            };
        }

        //self.validate();
    }

    // void b2DynamicTree::RemoveLeaf(int32 leaf)
    // {
    // 	if (leaf == m_root)
    // 	{
    // 		m_root = b2_nullNode;
    // 		return;
    // 	}

    // 	int32 parent = m_nodes[leaf].parent;
    // 	int32 grandParent = m_nodes[parent].parent;
    // 	int32 sibling;
    // 	if (m_nodes[parent].child1 == leaf)
    // 	{
    // 		sibling = m_nodes[parent].child2;
    // 	}
    // 	else
    // 	{
    // 		sibling = m_nodes[parent].child1;
    // 	}

    // 	if (grandParent != b2_nullNode)
    // 	{
    // 		// Destroy parent and connect sibling to grandParent.
    // 		if (m_nodes[grandParent].child1 == parent)
    // 		{
    // 			m_nodes[grandParent].child1 = sibling;
    // 		}
    // 		else
    // 		{
    // 			m_nodes[grandParent].child2 = sibling;
    // 		}
    // 		m_nodes[sibling].parent = grandParent;
    // 		FreeNode(parent);

    // 		// Adjust ancestor bounds.
    // 		int32 index = grandParent;
    // 		while (index != b2_nullNode)
    // 		{
    // 			index = Balance(index);

    // 			int32 child1 = m_nodes[index].child1;
    // 			int32 child2 = m_nodes[index].child2;

    // 			m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
    // 			m_nodes[index].height = 1 + b2Max(m_nodes[child1].height, m_nodes[child2].height);

    // 			index = m_nodes[index].parent;
    // 		}
    // 	}
    // 	else
    // 	{
    // 		m_root = sibling;
    // 		m_nodes[sibling].parent = b2_nullNode;
    // 		FreeNode(parent);
    // 	}

    // 	//Validate();
    // }

    // Perform a left or right rotation if node A is imbalanced.
    // Returns the new root index.
    fn balance(&mut self, i_a: usize) -> usize {
        let a = &mut self.nodes[i_a];
        if a.is_leaf() || a.height < 2 {
            return i_a;
        }

        let i_b = a.child1.unwrap();
        let i_c = a.child2.unwrap();
        let b = &mut self.nodes[i_b];
        let c = &mut self.nodes[i_c];

        let balance = c.height - b.height;

        // Rotate C up
        if balance > 1 {
            let i_f = c.child1.unwrap();
            let i_g = c.child2.unwrap();
            let f = &mut self.nodes[i_f];
            let g = &mut self.nodes[i_g];

            // Swap A and C
            c.child1 = Some(i_a);
            c.connection = a.connection;
            a.connection = TreeNodeConnection::Parent(i_c);

            // A's old parent should point to C
            if let TreeNodeConnection::Parent(i_a_parent) = c.connection {
                if self.nodes[i_a_parent].child1 == Some(i_a) {
                    self.nodes[i_a_parent].child1 = Some(i_c);
                } else {
                    self.nodes[i_a_parent].child2 = Some(i_c);
                }
            } else {
                self.root = Some(i_c);
            }

            // Rotate
            if f.height > g.height {
                c.child2 = Some(i_f);
                a.child2 = Some(i_g);
                g.connection = TreeNodeConnection::Parent(i_a);
                a.aabb = b.aabb.combine(&g.aabb);
                c.aabb = a.aabb.combine(&f.aabb);

                a.height = 1 + b.height.max(g.height);
                c.height = 1 + a.height.max(f.height);
            } else {
                c.child2 = Some(i_g);
                a.child2 = Some(i_f);
                f.connection = TreeNodeConnection::Parent(i_a);
                a.aabb = b.aabb.combine(&f.aabb);
                c.aabb = a.aabb.combine(&g.aabb);

                a.height = 1 + b.height.max(f.height);
                c.height = 1 + a.height.max(g.height);
            }

            return i_c;
        }

        // Rotate B up
        if balance < -1 {
            let i_d = b.child1.unwrap();
            let i_e = b.child2.unwrap();
            let d = &mut self.nodes[i_d];
            let e = &mut self.nodes[i_e];

            // Swap A and B
            b.child1 = Some(i_a);
            b.connection = a.connection;
            a.connection = TreeNodeConnection::Parent(i_b);

            // A's old parent should point to B
            if let TreeNodeConnection::Parent(i_a_parent) = b.connection {
                if self.nodes[i_a_parent].child1 == Some(i_a) {
                    self.nodes[i_a_parent].child1 = Some(i_b);
                } else {
                    self.nodes[i_a_parent].child2 = Some(i_b);
                }
            } else {
                self.root = Some(i_b);
            }

            // Rotate
            if d.height > e.height {
                b.child2 = Some(i_d);
                a.child1 = Some(i_e);
                e.connection = TreeNodeConnection::Parent(i_a);
                a.aabb = c.aabb.combine(&e.aabb);
                b.aabb = a.aabb.combine(&d.aabb);

                a.height = 1 + c.height.max(e.height);
                b.height = 1 + a.height.max(d.height);
            } else {
                b.child2 = Some(i_e);
                a.child1 = Some(i_d);
                d.connection = TreeNodeConnection::Parent(i_a);
                a.aabb = c.aabb.combine(&d.aabb);
                b.aabb = a.aabb.combine(&e.aabb);

                a.height = 1 + c.height.max(d.height);
                b.height = 1 + a.height.max(e.height);
            }

            return i_b;
        }

        i_a
    }
}
