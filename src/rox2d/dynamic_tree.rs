use crate::common::AABB_EXTENSION;

use super::{collision::Aabb, Vec2};

#[derive(Debug, Clone)]
enum TreeNodeConnection {
    None,
    /// Tree mode.
    Parent(usize),
    /// Linked-list mode.
    Next(usize),
}

/// A node in the dynamic tree. The client does not interact with this directly.
#[derive(Debug, Clone)]
pub struct TreeNode {
    aabb: Aabb,
    // user_data: Option<Box<dyn std::any::Any>>,
    parent: TreeNodeConnection,

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
#[derive(Debug, Clone)]
pub struct DynamicTree {
    root: Option<usize>,
    nodes: Vec<TreeNode>,
    // node_count: usize,
    // node_capacity: usize,
    free_list: Vec<TreeNode>,
    insertion_count: usize,
}

impl DynamicTree {
    pub fn new() -> Self {
        let mut nodes = Vec::with_capacity(16);
        for i in 0..nodes.capacity() - 1 {
            nodes.push(TreeNode {
                aabb: Aabb::default(),
                parent: TreeNodeConnection::Next(i + 1),
                child1: None,
                child2: None,
                height: -1,
                moved: false,
            });
        }
        nodes.push(TreeNode {
            aabb: Aabb::default(),
            parent: TreeNodeConnection::None,
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

    // Create a proxy in the tree as a leaf node. We return the index
    // of the node instead of a pointer so that we can grow
    // the node pool.
    // pub fn create_proxy(&mut self, aabb: &Aabb, user_data: usize) -> usize {
    pub fn create_proxy(&mut self, aabb: &Aabb) -> usize {
        // Fatten the aabb.
        let r = Vec2::new(AABB_EXTENSION, AABB_EXTENSION);

        let proxy_id = self.nodes.len();
        self.nodes.push(TreeNode {
            aabb: Aabb {
                lower_bound: aabb.lower_bound - r,
                upper_bound: aabb.upper_bound + r,
            },
            parent: TreeNodeConnection::None,
            child1: None,
            child2: None,
            height: 0,
            moved: true,
        });

        self.insert_leaf(proxy_id);

        proxy_id
    }

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
            self.nodes[leaf].parent = TreeNodeConnection::None;
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
        let old_parent = self.nodes[sibling].parent;
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
            self.nodes[sibling].parent =
                TreeNodeConnection::Parent(new_parent);
            self.nodes[leaf].parent =
                TreeNodeConnection::Parent(new_parent);
        } else {
            // The sibling was the root.
            self.nodes[new_parent].child1 = sibling;
            self.nodes[new_parent].child2 = leaf;
            self.nodes[sibling].parent =
                TreeNodeConnection::Parent(new_parent);
            self.nodes[leaf].parent =
                TreeNodeConnection::Parent(new_parent);
            self.root = Some(new_parent);
        }

        // Walk back up the tree fixing heights and AABBs
        let mut connection = self.nodes[leaf].parent;
        while let TreeNodeConnection::Parent(index) = connection {
            index = self.balance(index);

            let child1 = self.nodes[index].child1.unwrap();
            let child2 = self.nodes[index].child2.unwrap();

            self.nodes[index].height =
                1 + self.nodes[child1].height.max(self.nodes[child2].height);
            self.nodes[index].aabb =
                self.nodes[child1].aabb.combine(&self.nodes[child2].aabb);

            connection = self.nodes[index].parent;
        }

        //self.validate();
    }

    pub fn remove_leaf(&mut self, leaf: usize) {
        if self.root == Some(leaf){
            self.root = None;
            return;
        }

        let parent = match self.nodes[leaf].parent {
            TreeNodeConnection::Parent(parent) => parent,
            _ => panic!("No parent found"),
        };
        let sibling = if self.nodes[parent].child1 == Some(leaf) {
            self.nodes[parent].child2
        } else {
            self.nodes[parent].child1
        };

        if let TreeNodeConnection::Parent(grand_parent) =
            self.nodes[parent].parent
        {
            // Destroy parent and connect sibling to grand_parent.
            if self.nodes[grand_parent].child1 == Some(parent) {
                self.nodes[grand_parent].child1 = sibling;
            } else {
                self.nodes[grand_parent].child2 = sibling;
            }
            self.nodes[sibling.unwrap()].parent =
                TreeNodeConnection::Parent(grand_parent);
            self.free_node(parent);

            // Adjust ancestor bounds.
            let mut index = grand_parent;
            while let TreeNodeConnection::Parent(index) = {
                index = self.balance(index);

                let child1 = self.nodes[index].child1.unwrap();
                let child2 = self.nodes[index].child2.unwrap();

                self.nodes[index].aabb =
                    self.nodes[child1].aabb.combine(&self.nodes[child2].aabb);
                self.nodes[index].height = 1 + self.nodes[child1]
                    .height
                    .max(self.nodes[child2].height);

                index = match self.nodes[index].parent {
                    TreeNodeConnection::Parent(index) => index,
                    _ => panic!("No parent found"),
                };
            }
        } else {
            self.root = sibling;
            self.free_node(parent);
            if let Some(sibling) = sibling {
            self.nodes[sibling].parent = TreeNodeConnection::None;
            }
        }

        //self.validate();
    }

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
            c.parent = a.parent;
            a.parent = TreeNodeConnection::Parent(i_c);

            // A's old parent should point to C
            if let TreeNodeConnection::Parent(i_a_parent) = c.parent {
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
                g.parent = TreeNodeConnection::Parent(i_a);
                a.aabb = b.aabb.combine(&g.aabb);
                c.aabb = a.aabb.combine(&f.aabb);

                a.height = 1 + b.height.max(g.height);
                c.height = 1 + a.height.max(f.height);
            } else {
                c.child2 = Some(i_g);
                a.child2 = Some(i_f);
                f.parent = TreeNodeConnection::Parent(i_a);
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
            b.parent = a.parent;
            a.parent = TreeNodeConnection::Parent(i_b);

            // A's old parent should point to B
            if let TreeNodeConnection::Parent(i_a_parent) = b.parent {
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
                e.parent = TreeNodeConnection::Parent(i_a);
                a.aabb = c.aabb.combine(&e.aabb);
                b.aabb = a.aabb.combine(&d.aabb);

                a.height = 1 + c.height.max(e.height);
                b.height = 1 + a.height.max(d.height);
            } else {
                b.child2 = Some(i_e);
                a.child1 = Some(i_d);
                d.parent = TreeNodeConnection::Parent(i_a);
                a.aabb = c.aabb.combine(&d.aabb);
                b.aabb = a.aabb.combine(&e.aabb);

                a.height = 1 + c.height.max(d.height);
                b.height = 1 + a.height.max(e.height);
            }

            return i_b;
        }

        i_a
    }

    pub fn free_node(&mut self, index: usize) {
        let node = self.nodes.remove(index);
        node.parent = TreeNodeConnection::None;
        node.height = -1;
        self.free_list.push(node);
    }

    pub fn query(&self, aabb: Aabb, mut callback: impl FnMut(usize)) {
        let mut stack = Vec::new();
        stack.push(self.root);

        while let Some(index) = stack.pop() {
            if let Some(index) = index {
                let node = &self.nodes[index];

                if node.aabb.overlaps(aabb) {
                    if node.is_leaf() {
                        callback(index);
                    } else {
                        stack.push(node.child1);
                        stack.push(node.child2);
                    }
                }
            }
        }
    }
}
