use super::{broad_phase::BroadPhase, contact::Contact};

pub struct ContactManager {
    pub broad_phase: BroadPhase,
    pub contact_list: Vec<Contact>,
    // pub contact_filter: Box<dyn ContactFilter>,
    // pub contact_listener: Box<dyn ContactListener>,
}

impl ContactManager {
    pub fn new() -> Self {
        Self {
            broad_phase: BroadPhase::new(),
            contact_list: Vec::new(),
            // contact_count: 0,
            // contact_filter: Box::new(DefaultContactFilter),
            // contact_listener: Box::new(DefaultContactListener),
        }
    }

    fn add_pair(&mut self, proxy_id_a: usize, proxy_id_b: usize) {
        let fixture_a = self.broad_phase.tree.get_user_data(proxy_id_a);
        let fixture_b = self.broad_phase.tree.get_user_data(proxy_id_b);

        let body_a = fixture_a.body;
        let body_b = fixture_b.body;

        // Are the fixtures on the same body?
        if body_a == body_b {
            return;
        }

        // TODO_ERIN use a hash table to remove a potential bottleneck when both
        // bodies have a lot of contacts.
        // Does a contact already exist?
        let mut edge = body_b.contact_list;
        while let Some(edge) = edge {
            if edge.other == body_a {
                let f_a = edge.contact.fixture_a;
                let f_b = edge.contact.fixture_b;
                let i_a = edge.contact.child_index_a;
                let i_b = edge.contact.child_index_b;

                if f_a == fixture_a
                    && f_b == fixture_b
                    && i_a == fixture_a.child_index
                    && i_b == fixture_b.child_index
                {
                    // A contact already exists.
                    return;
                }

                if f_a == fixture_b
                    && f_b == fixture_a
                    && i_a == fixture_b.child_index
                    && i_b == fixture_a.child_index
                {
                    // A contact already exists.
                    return;
                }
            }

            edge = edge.next;
        }

        // Does a joint override collision? Is at least one body dynamic?
        if body_b.should_collide(body_a) == false {
            return;
        }

        // Check user filtering.
        // if m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false
        // {
        // 	return;
        // }

        // Call the factory.
        let contact = Contact::new(fixture_a, fixture_b);
        if contact.is_none() {
            return;
        }

        // Contact creation may swap fixtures.
        let fixture_a = contact.unwrap().fixture_a;
        let fixture_b = contact.unwrap().fixture_b;

        // Insert into the world.
        contact.unwrap().prev = None;
        contact.unwrap().next = self.contact_list.first();
        if let Some(contact) = self.contact_list.first() {
            contact.prev = Some(contact);
        }
        self.contact_list.insert(0, contact.unwrap());

        // Connect to island graph.

        // Connect to body A
        let mut node_a = contact.unwrap().node_a;
        node_a.contact = contact.unwrap();
        node_a.other = body_b;

        node_a.prev = None;
        node_a.next = body_a.contact_list;
        if let Some(node_a) = body_a.contact_list {
            node_a.prev = Some(node_a);
        }
        body_a.contact_list = Some(node_a);

        // Connect to body B
        let mut node_b = contact.unwrap().node_b;
        node_b.contact = contact.unwrap();
        node_b.other = body_a;

        node_b.prev = None;
        node_b.next = body_b.contact_list;
        if let Some(node_b) = body_b.contact_list {
            node_b.prev = Some(node_b);
        }
        body_b.contact_list = Some(node_b);
    }
}
