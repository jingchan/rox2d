// bool b2PolygonShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
// {
// 	b2Vec2 pLocal = b2MulT(xf.q, p - xf.p);

// 	for (int32 i = 0; i < m_count; ++i)
// 	{
// 		float dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
// 		if (dot > 0.0f)
// 		{
// 			return false;
// 		}
// 	}

// 	return true;
// }

/// This characterizes how forces get applied.
struct MassData {
    /// The mass, in kilograms.
    mass: f32,

    /// The rotational inertia of the shape.
    /// TODO: Do we need to say "about the local origin".
    inertia: f32,

    /// The centroid relative to the origin.
    center: Vec2,
}
