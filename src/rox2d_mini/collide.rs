use super::{
    body::Body,
    math::{Mat2x2, Vec2},
};

const RELATIVE_TOLERANCE: f32 = 0.95;
const ABSOLUTE_TOLERANCE: f32 = 0.01;

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct FeaturePair {
    in_edge1: EdgeNum,
    out_edge1: EdgeNum,
    in_edge2: EdgeNum,
    out_edge2: EdgeNum,
}
// pub enum FeaturePair {
//     Edges {
//         inEdge1: EdgeNum,
//         outEdge1: EdgeNum,
//         inEdge2: EdgeNum,
//         outEdge2: EdgeNum,
//     },
//     Value(i32),
// }

// impl Default for FeaturePair {
//     fn default() -> Self {
//         Self::Value(0)
//     }
// }

impl FeaturePair {
    // pub fn new() -> Self {
    //     Self::default()
    // }

    pub fn flip(&mut self) {
        std::mem::swap(&mut self.in_edge1, &mut self.in_edge2);
        std::mem::swap(&mut self.out_edge1, &mut self.out_edge2);
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct Contact {
    pub position: Vec2,
    pub normal: Vec2,
    pub r1: Vec2,
    pub r2: Vec2,
    pub separation: f32,
    /// Accumulated normal impulse (Pn)
    pub accumulated_impulse_normal: f32,
    /// Accumulated tangent impulse (Pt)
    pub accumulated_impulse_tangent: f32,
    /// Accumulated normal impulse for position bias (Pnb)
    pub accumulated_impulse_normal_biasnb: f32,
    pub mass_normal: f32,
    pub mass_tangent: f32,
    pub bias: f32,
    pub feature_pair: FeaturePair,
}

#[derive(Debug, Clone, Copy)]
enum Axis {
    FaceAX,
    FaceAY,
    FaceBX,
    FaceBY,
}

// Box vertex and edge numbering:
//
//        ^ y
//        |
//        e1
//   v2 ------ v1
//    |        |
// e2 |        | e4  --> x
//    |        |
//   v3 ------ v4
//        e3
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum EdgeNum {
    #[default]
    NoEdge,
    Edge1,
    Edge2,
    Edge3,
    Edge4,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ClipVertex {
    v: Vec2,
    feature_pair: FeaturePair,
}

pub fn clip_segment_to_line(
    v_out: &mut [ClipVertex; 2],
    v_in: &[ClipVertex; 2],
    normal: Vec2,
    offset: f32,
    clip_edge: EdgeNum,
) -> usize {
    // Start with no output points
    let mut num_out = 0;

    // Calculate the distance of end points to the line
    let distance0 = normal.dot(v_in[0].v) - offset;
    let distance1 = normal.dot(v_in[1].v) - offset;

    // If the points are behind the plane
    if distance0 <= 0.0 {
        v_out[num_out] = v_in[0];
        num_out += 1;
    }
    if distance1 <= 0.0 {
        v_out[num_out] = v_in[1];
        num_out += 1;
    }

    // If the points are on different sides of the plane
    if distance0 * distance1 < 0.0 {
        // Find intersection point of edge and plane
        let interp = distance0 / (distance0 - distance1);
        v_out[num_out].v = v_in[0].v + interp * (v_in[1].v - v_in[0].v);

        // TODO: Fixup
        v_out[num_out].feature_pair = if distance0 > 0.0 {
            FeaturePair {
                in_edge1: clip_edge,
                in_edge2: EdgeNum::NoEdge,
                ..v_in[0].feature_pair
            }
        } else {
            FeaturePair {
                out_edge1: clip_edge,
                out_edge2: EdgeNum::NoEdge,
                ..v_in[1].feature_pair
            }
        };
        num_out += 1;
    }

    num_out
}

fn compute_incident_edge(
    c: &mut [ClipVertex; 2],
    h: Vec2,
    pos: Vec2,
    rot: Mat2x2,
    normal: Vec2,
) {
    // The normal is from the reference box. Convert it
    // to the incident boxe's frame and flip sign.
    let rot_t = rot.transpose();
    let n = -(rot_t * normal);
    let n_abs = n.abs();

    if n_abs.x > n_abs.y {
        if n.x > 0.0 {
            [
                ClipVertex {
                    v: pos + rot * Vec2::new(h.x, -h.y),
                    feature_pair: FeaturePair {
                        in_edge2: EdgeNum::Edge3,
                        out_edge2: EdgeNum::Edge4,
                        ..c[0].feature_pair
                    },
                },
                ClipVertex {
                    v: pos + rot * Vec2::new(h.x, h.y),
                    feature_pair: FeaturePair {
                        in_edge2: EdgeNum::Edge4,
                        out_edge2: EdgeNum::Edge1,
                        ..c[1].feature_pair
                    },
                },
            ];
        } else {
            [
                ClipVertex {
                    v: pos + rot * Vec2::new(-h.x, h.y),
                    feature_pair: FeaturePair {
                        in_edge2: EdgeNum::Edge1,
                        out_edge2: EdgeNum::Edge2,
                        ..c[0].feature_pair
                    },
                },
                ClipVertex {
                    v: pos + rot * Vec2::new(-h.x, -h.y),
                    feature_pair: FeaturePair {
                        in_edge2: EdgeNum::Edge2,
                        out_edge2: EdgeNum::Edge3,
                        ..c[1].feature_pair
                    },
                },
            ];
        }
    } else {
        if n.y > 0.0 {
            [
                ClipVertex {
                    v: pos + rot * Vec2::new(h.x, h.y),
                    feature_pair: FeaturePair {
                        in_edge2: EdgeNum::Edge4,
                        out_edge2: EdgeNum::Edge1,
                        ..c[0].feature_pair
                    },
                },
                ClipVertex {
                    v: pos + rot * Vec2::new(-h.x, h.y),
                    feature_pair: FeaturePair {
                        in_edge2: EdgeNum::Edge1,
                        out_edge2: EdgeNum::Edge2,
                        ..c[1].feature_pair
                    },
                },
            ];
        } else {
            [
                ClipVertex {
                    v: pos + rot * Vec2::new(-h.x, -h.y),
                    feature_pair: FeaturePair {
                        in_edge2: EdgeNum::Edge2,
                        out_edge2: EdgeNum::Edge3,
                        ..c[0].feature_pair
                    },
                },
                ClipVertex {
                    v: pos + rot * Vec2::new(h.x, -h.y),
                    feature_pair: FeaturePair {
                        in_edge2: EdgeNum::Edge3,
                        out_edge2: EdgeNum::Edge4,
                        ..c[1].feature_pair
                    },
                },
            ];
        }
    }
}

pub fn collide(
    contacts: &mut [Contact],
    body_a: &Body,
    body_b: &Body,
) -> usize {
    let h_a = 0.5 * body_a.size;
    let h_b = 0.5 * body_b.size;

    let pos_a = body_a.position;
    let pos_b = body_b.position;

    let rot_a = Mat2x2::from_angle(body_a.rotation);

    let rot_b = Mat2x2::from_angle(body_b.rotation);

    let rot_a_t = rot_a.transpose();
    let rot_b_t = rot_b.transpose();

    let dp = pos_b - pos_a;
    let d_a = rot_a_t * dp;
    let d_b = rot_b_t * dp;

    let c = rot_a_t * rot_b;

    let abs_c = c.abs();
    let abs_c_t = abs_c.transpose();

    // Box A faces
    let face_a = d_a.abs() - h_a - abs_c * h_b;
    if face_a.x > 0.0 || face_a.y > 0.0 {
        return 0;
    }
    // Box B faces
    let face_b = d_b.abs() - abs_c_t * h_a - h_b;
    if face_b.x > 0.0 || face_b.y > 0.0 {
        return 0;
    }

    // Find best axis
    let mut axis;
    let mut separation;
    let mut normal;

    // Box A faces
    axis = Axis::FaceAX;
    separation = face_a.x;
    normal = if d_a.x > 0.0 { rot_a.col1 } else { -rot_a.col1 };

    if face_a.y > RELATIVE_TOLERANCE * separation + ABSOLUTE_TOLERANCE * h_a.y {
        axis = Axis::FaceAY;
        separation = face_a.y;
        normal = if d_a.y > 0.0 { rot_a.col2 } else { -rot_a.col2 };
    }

    // Box B faces
    if face_b.x > RELATIVE_TOLERANCE * separation + ABSOLUTE_TOLERANCE * h_b.x {
        axis = Axis::FaceBX;
        separation = face_b.x;
        normal = if d_b.x > 0.0 { rot_b.col1 } else { -rot_b.col1 };
    }

    if face_b.y > RELATIVE_TOLERANCE * separation + ABSOLUTE_TOLERANCE * h_b.y {
        axis = Axis::FaceBY;
        separation = face_b.y;
        normal = if d_b.y > 0.0 { rot_b.col2 } else { -rot_b.col2 };
    }

    // Setup clipping plane data based on the separating axis
    let mut front_normal;
    let mut side_normal;
    let mut incident_edge = [ClipVertex::default(); 2];
    let mut front;
    let mut neg_side;
    let mut pos_side;
    let mut neg_edge;
    let mut pos_edge;

    // Compute the clipping lines and the line segment to be clipped.
    match axis {
        Axis::FaceAX => {
            front_normal = normal;
            front = pos_a.dot(front_normal) + h_a.x;
            side_normal = rot_a.col2;
            let side = pos_a.dot(side_normal);
            neg_side = -side + h_a.y;
            pos_side = side + h_a.y;
            neg_edge = EdgeNum::Edge3;
            pos_edge = EdgeNum::Edge1;
            compute_incident_edge(
                &mut incident_edge,
                h_b,
                pos_b,
                rot_b,
                front_normal,
            );
        }
        Axis::FaceAY => {
            front_normal = normal;
            front = pos_a.dot(front_normal) + h_a.y;
            side_normal = rot_a.col1;
            let side = pos_a.dot(side_normal);
            neg_side = -side + h_a.x;
            pos_side = side + h_a.x;
            neg_edge = EdgeNum::Edge2;
            pos_edge = EdgeNum::Edge4;
            compute_incident_edge(
                &mut incident_edge,
                h_b,
                pos_b,
                rot_b,
                front_normal,
            );
        }
        Axis::FaceBX => {
            front_normal = -normal;
            front = pos_b.dot(front_normal) + h_b.x;
            side_normal = rot_b.col2;
            let side = pos_b.dot(side_normal);
            neg_side = -side + h_b.y;
            pos_side = side + h_b.y;
            neg_edge = EdgeNum::Edge3;
            pos_edge = EdgeNum::Edge1;
            compute_incident_edge(
                &mut incident_edge,
                h_a,
                pos_a,
                rot_a,
                front_normal,
            );
        }
        Axis::FaceBY => {
            front_normal = -normal;
            front = pos_b.dot(front_normal) + h_b.y;
            side_normal = rot_b.col1;
            let side = pos_b.dot(side_normal);
            neg_side = -side + h_b.x;
            pos_side = side + h_b.x;
            neg_edge = EdgeNum::Edge2;
            pos_edge = EdgeNum::Edge4;
            compute_incident_edge(
                &mut incident_edge,
                h_a,
                pos_a,
                rot_a,
                front_normal,
            );
        }
    };

    // clip other face with 5 box planes (1 face plane, 4 edge planes)
    let mut clip_points1 = [ClipVertex::default(); 2];
    let mut clip_points2 = [ClipVertex::default(); 2];

    // Clip to box side 1
    let mut np = clip_segment_to_line(
        &mut clip_points1,
        &mut incident_edge,
        -side_normal,
        neg_side,
        neg_edge,
    );

    if np < 2 {
        return 0;
    }

    // Clip to negative box side 1
    np = clip_segment_to_line(
        &mut clip_points2,
        &mut clip_points1,
        side_normal,
        pos_side,
        pos_edge,
    );

    if np < 2 {
        return 0;
    }

    // Now clip_points2 contains the clipped points.
    // Due to roundoff, it is possible that clipping removes all points.

    let mut num_contacts = 0;
    for i in 0..np {
        let separation = front_normal.dot(clip_points2[i].v) - front;

        if separation <= 0.0 {
            contacts[num_contacts].separation = separation;
            contacts[num_contacts].normal = normal;
            // slide contact point onto reference face (easy to cull)
            contacts[num_contacts].position =
                clip_points2[i].v - separation * front_normal;
            contacts[num_contacts].feature_pair = clip_points2[i].feature_pair;
            num_contacts += 1;
        }
    }

    num_contacts
}
