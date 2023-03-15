use crate::{
    body::{Body, BodyType},
    contact::ContactFlags,
};

use super::{broad_phase::BroadPhase, contact::Contact};

#[derive(Debug, Clone)]
pub struct ContactManager {
    pub broad_phase: BroadPhase,
    pub contacts: Vec<Contact>,
    pub contact_list: Vec<Contact>,
    // pub contact_filter: Box<dyn ContactFilter>,
    // pub contact_listener: Box<dyn ContactListener>,
}

impl ContactManager {
    pub fn new() -> Self {
        Self {
            broad_phase: BroadPhase::new(),
            contacts: Vec::new(),
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

    // This is the top level collision call for the time step. Here
    // all the narrow phase collision is processed for the world
    // contact list.
    pub fn collide(&mut self) {
        // Update awake contacts.
        // self.contacts = self.contacts.drain_filter(|contact| {});
        for contact in &mut self.contacts {
            let fixture_a = contact.fixture_a;
            let fixture_b = contact.fixture_b;
            let child_index_a = contact.child_index_a;
            let child_index_b = contact.child_index_b;
            // need to find a way to provide body here, swisstable?
            let body_a: &Body = fixture_a.body;
            let body_b: &Body = fixture_b.body;

            // Is this contact flagged for filtering?
            if contact.flags.contains(ContactFlags::FILTER) {
                // Should these bodies collide?
                if body_b.should_collide(body_a) == false {
                    // remove and continue
                    continue;
                }

                // // Check user filtering.
                // if self.contact_filter.is_some() && contact_filter.should_collide(fixture_a, fixture_b) == false
                // {
                //     // remove and continue
                //     continue;
                // }

                // Clear the filtering flag.
                contact.flags.remove(ContactFlags::FILTER);
            }

            // At least one body should be awake and it must be dynamic or
            // kinematic.
            let body_a_active =
                body_a.is_awake() && body_a.type_ != BodyType::Static;
            let body_b_active =
                body_b.is_awake() && body_b.type_ != BodyType::Static;
            if !body_a_active && !body_b_active {
                continue;
            }

            // Here we destroy contacts that cease to overlap in the broad-phase.
            let mut proxy_id_a =
                fixture_a.proxies.get(child_index_a).unwrap().proxy_id;
            let mut proxy_id_b =
                fixture_b.proxies.get(child_index_b).unwrap().proxy_id;
            let mut overlap =
                self.broad_phase.tree.test_overlap(proxy_id_a, proxy_id_b);
            if !overlap {
                // self.destroy(contact);
                // rmeove and ocntinue
                continue;
            }

            // The contact persists.
            contact.update();
        }
    }
    // void b2ContactManager::Collide()
    // {
    // 	// Update awake contacts.
    // 	b2Contact* c = m_contactList;
    // 	while (c)
    // 	{
    // 		b2Fixture* fixtureA = c->GetFixtureA();
    // 		b2Fixture* fixtureB = c->GetFixtureB();
    // 		int32 indexA = c->GetChildIndexA();
    // 		int32 indexB = c->GetChildIndexB();
    // 		b2Body* bodyA = fixtureA->GetBody();
    // 		b2Body* bodyB = fixtureB->GetBody();

    // 		// Is this contact flagged for filtering?
    // 		if (c->m_flags & b2Contact::e_filterFlag)
    // 		{
    // 			// Should these bodies collide?
    // 			if (bodyB->ShouldCollide(bodyA) == false)
    // 			{
    // 				b2Contact* cNuke = c;
    // 				c = cNuke->GetNext();
    // 				Destroy(cNuke);
    // 				continue;
    // 			}

    // 			// Check user filtering.
    // 			if (m_contactFilter && m_contactFilter->ShouldCollide(fixtureA, fixtureB) == false)
    // 			{
    // 				b2Contact* cNuke = c;
    // 				c = cNuke->GetNext();
    // 				Destroy(cNuke);
    // 				continue;
    // 			}

    // 			// Clear the filtering flag.
    // 			c->m_flags &= ~b2Contact::e_filterFlag;
    // 		}

    // 		bool activeA = bodyA->IsAwake() && bodyA->m_type != b2_staticBody;
    // 		bool activeB = bodyB->IsAwake() && bodyB->m_type != b2_staticBody;

    // 		// At least one body must be awake and it must be dynamic or kinematic.
    // 		if (activeA == false && activeB == false)
    // 		{
    // 			c = c->GetNext();
    // 			continue;
    // 		}

    // 		int32 proxyIdA = fixtureA->m_proxies[indexA].proxyId;
    // 		int32 proxyIdB = fixtureB->m_proxies[indexB].proxyId;
    // 		bool overlap = m_broadPhase.TestOverlap(proxyIdA, proxyIdB);

    // 		// Here we destroy contacts that cease to overlap in the broad-phase.
    // 		if (overlap == false)
    // 		{
    // 			b2Contact* cNuke = c;
    // 			c = cNuke->GetNext();
    // 			Destroy(cNuke);
    // 			continue;
    // 		}

    // 		// The contact persists.
    // 		c->Update(m_contactListener);
    // 		c = c->GetNext();
    // 	}
    // }
    pub fn find_new_contacts(&mut self) {
        self.broad_phase
            .update_pairs(&mut |a, b| self.add_pair(a, b));
    }
}
