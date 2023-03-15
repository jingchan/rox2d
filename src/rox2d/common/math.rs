pub use std::mem::swap;
use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Copy, Clone, Debug)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self::splat(0.0);
    pub const ONE: Self = Self::splat(1.0);

    #[inline(always)]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    #[inline(always)]
    pub const fn splat(value: f32) -> Self {
        Self { x: value, y: value }
    }

    #[inline(always)]
    pub const fn get(&self, index: usize) -> f32 {
        match index {
            0 => self.x,
            1 => self.y,
            _ => panic!("index out of bounds"),
        }
    }

    #[inline(always)]
    pub const fn get_mut(&mut self, index: usize) -> &mut f32 {
        match index {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("index out of bounds"),
        }
    }

    #[inline]
    pub fn length(&self) -> f32 {
        self.x.hypot(self.y)
    }

    #[inline]
    pub fn dot(&self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    #[inline]
    pub fn cross(&self, other: Self) -> f32 {
        self.x * other.y - self.y * other.x
    }

    #[inline]
    pub fn cross_scalar(&self, scalar: f32) -> Self {
        Self {
            x: scalar * self.y,
            y: -scalar * self.x,
        }
    }

    #[inline]
    pub fn scalar_cross(scalar: f32, vec: Self) -> Self {
        Self {
            x: -scalar * vec.y,
            y: scalar * vec.x,
        }
    }

    #[inline]
    pub fn normalize(&self) -> Self {
        let len = self.length();
        assert!(len != 0.0);
        Self {
            x: self.x / len,
            y: self.y / len,
        }
    }

    #[inline]
    pub fn rotate(&self, angle: f32) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Self {
            x: self.x * c - self.y * s,
            y: self.x * s + self.y * c,
        }
    }

    #[inline]
    pub fn mul_scalar(&self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
        }
    }

    #[inline]
    pub fn abs(&self) -> Self {
        Self {
            x: self.x.abs(),
            y: self.y.abs(),
        }
    }

    #[inline]
    pub fn min(&self, other: Self) -> Self {
        Self {
            x: self.x.min(other.x),
            y: self.y.min(other.y),
        }
    }

    #[inline]
    pub fn max(&self, other: Self) -> Self {
        Self {
            x: self.x.max(other.x),
            y: self.y.max(other.y),
        }
    }

    #[inline]
    pub fn clamp(&self, min: Self, max: Self) -> Self {
        self.max(min).min(max)
    }
}

// inline Vec2 operator * (const Mat22& A, const Vec2& v)
// {
// 	return Vec2(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
// }

// inline Vec2 operator * (float s, const Vec2& v)
// {
// 	return Vec2(s * v.x, s * v.y);
// }

// inline Mat22 operator + (const Mat22& A, const Mat22& B)
// {
// 	return Mat22(A.col1 + B.col1, A.col2 + B.col2);
// }

// inline Mat22 operator * (const Mat22& A, const Mat22& B)
// {
// 	return Mat22(A * B.col1, A * B.col2);
// }

// inline float Abs(float a)
// {
// 	return a > 0.0f ? a : -a;
// }

// inline Vec2 Abs(const Vec2& a)
// {
// 	return Vec2(fabsf(a.x), fabsf(a.y));
// }

impl Default for Vec2 {
    #[inline(always)]
    fn default() -> Self {
        Self { x: 0.0, y: 0.0 }
    }
}

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, other: Self) {
        self.x += other.x;
        self.y += other.y;
    }
}

impl Neg for Vec2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, other: Self) {
        self.x -= other.x;
        self.y -= other.y;
    }
}

impl Mul for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self {
            x: self.x * other.x,
            y: self.y * other.y,
        }
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, other: f32) -> Self {
        Self {
            x: self.x * other,
            y: self.y * other,
        }
    }
}

impl Mul<Vec2> for f32 {
    type Output = Vec2;
    #[inline]
    fn mul(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: self * other.x,
            y: self * other.y,
        }
    }
}

impl MulAssign for Vec2 {
    #[inline]
    fn mul_assign(&mut self, other: Self) {
        self.x *= other.x;
        self.y *= other.y;
    }
}

impl MulAssign<f32> for Vec2 {
    #[inline]
    fn mul_assign(&mut self, other: f32) {
        self.x *= other;
        self.y *= other;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Mat2x2 {
    pub col1: Vec2,
    pub col2: Vec2,
}

impl Mat2x2 {
    pub const ZERO: Mat2x2 = Mat2x2 {
        col1: Vec2::ZERO,
        col2: Vec2::ZERO,
    };

    pub const ONE: Mat2x2 = Mat2x2 {
        col1: Vec2::ONE,
        col2: Vec2::ONE,
    };

    pub const IDENTITY: Mat2x2 = Mat2x2::from_diag(Vec2::ONE);

    #[inline(always)]
    pub const fn new(m00: f32, m01: f32, m10: f32, m11: f32) -> Self {
        Self {
            col1: Vec2::new(m00, m10),
            col2: Vec2::new(m01, m11),
        }
    }

    #[inline(always)]
    pub const fn from_cols(col1: Vec2, col2: Vec2) -> Self {
        Self { col1, col2 }
    }

    #[inline(always)]
    pub const fn from_diag(diag: Vec2) -> Self {
        Self {
            col1: Vec2::new(diag.x, 0.0),
            col2: Vec2::new(0.0, diag.y),
        }
    }

    /// Construct a 2x2 matrix from 2x1 row vectors.
    ///
    /// Note: This allows for specification of the matrix in row-major order for
    /// stylistic and visual clarity, as the data will ultimately be stored in
    /// column-major order.
    #[inline(always)]
    pub const fn from_rows(row1: Vec2, row2: Vec2) -> Self {
        Self {
            col1: Vec2::new(row1.x, row2.x),
            col2: Vec2::new(row1.y, row2.y),
        }
    }

    #[inline(always)]
    pub fn from_angle(angle: f32) -> Self {
        let c = angle.cos();
        let s = angle.sin();
        Self {
            col1: Vec2::new(c, s),
            col2: Vec2::new(-s, c),
        }
    }

    #[inline]
    pub const fn transpose(&self) -> Self {
        Self {
            col1: Vec2::new(self.col1.x, self.col2.x),
            col2: Vec2::new(self.col1.y, self.col2.y),
        }
    }

    #[inline]
    pub fn determinant(&self) -> f32 {
        self.col1.x * self.col2.y - self.col2.x * self.col1.y
    }

    #[inline]
    pub fn invert(&self) -> Self {
        let inv_det = {
            let det = self.determinant();
            assert!(det != 0.0);
            det.recip()
        };
        Self::new(
            inv_det * self.col2.y,
            -inv_det * self.col2.x,
            -inv_det * self.col1.y,
            inv_det * self.col1.x,
        )
    }

    #[inline]
    pub fn abs(&self) -> Self {
        Self {
            col1: self.col1.abs(),
            col2: self.col2.abs(),
        }
    }

    #[inline]
    pub fn min(&self, other: Self) -> Self {
        Self {
            col1: self.col1.min(other.col1),
            col2: self.col2.min(other.col2),
        }
    }

    #[inline]
    pub fn max(&self, other: Self) -> Self {
        Self {
            col1: self.col1.max(other.col1),
            col2: self.col2.max(other.col2),
        }
    }
}

impl Add for Mat2x2 {
    type Output = Self;
    #[inline]
    fn add(self, other: Self) -> Self {
        Self {
            col1: self.col1 + other.col1,
            col2: self.col2 + other.col2,
        }
    }
}

impl Mul for Mat2x2 {
    type Output = Self;
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self {
            col1: self * other.col1,
            col2: self * other.col2,
        }
    }
}

impl Mul<Vec2> for Mat2x2 {
    type Output = Vec2;
    #[inline]
    fn mul(self, other: Vec2) -> Vec2 {
        let Mat2x2 {
            col1: row1,
            col2: row2,
        } = self.transpose();
        Vec2::new(row1.dot(other), row2.dot(other))
    }
}

/// Random number in range [-1,1]
///
/// TODO: Could try to make faster using rand::thread_rng().
/// TODO: Try 2.0*rand::Rng::gen_range(-1.0..1.0)
#[inline(always)]
fn rand() -> f32 {
    2.0 * rand::random::<f32>() - 1.0

    // 2.0*rand::Rng::gen_range(-1.0..1.0)
}

/// Random number in range [-1,1]
///
/// TODO: Try to make faster using rand::thread_rng().
/// TODO: Try 2.0*rand::Rng::gen_range(lo..hi)
#[inline(always)]
fn rand_between(lo: f32, hi: f32) -> f32 {
    rand::random::<f32>() * (hi - lo) + lo
}

#[derive(Copy, Clone, Debug)]
pub struct Rot {
    /// Sine
    s: f32,
    /// Cosine
    c: f32,
}

impl Rot {
    /// Construct a new rotation from an angle in radians.
    #[inline(always)]
    pub fn new(angle: f32) -> Self {
        Self {
            s: angle.sin(),
            c: angle.cos(),
        }
    }

    /// Set this to the identity rotation.
    #[inline(always)]
    pub fn set_identity(&mut self) {
        self.s = 0.0;
        self.c = 1.0;
    }

    /// Get the angle in radians.
    #[inline]
    pub fn get_angle(&self) -> f32 {
        self.s.atan2(self.c)
    }

    /// Set using an angle in radians.
    #[inline]
    pub fn set_angle(&mut self, angle: f32) {
        self.s = angle.sin();
        self.c = angle.cos();
    }

    /// Get the X-axis
    #[inline]
    pub fn get_x_axis(&self) -> Vec2 {
        Vec2::new(self.c, self.s)
    }

    /// Get the Y-axis
    #[inline]
    pub fn get_y_axis(&self) -> Vec2 {
        Vec2::new(-self.s, self.c)
    }

    /// Get the inverse of this rotation.
    #[inline]
    pub fn get_inverse(&self) -> Self {
        Self {
            s: -self.s,
            c: self.c,
        }
    }
}

impl Mul for Rot {
    type Output = Self;
    #[inline]
    fn mul(self, other: Self) -> Self {
        Self {
            s: self.s * other.c + self.c * other.s,
            c: self.c * other.c - self.s * other.s,
        }
    }
}

impl Mul<Vec2> for Rot {
    type Output = Vec2;
    #[inline]
    fn mul(self, other: Vec2) -> Vec2 {
        Vec2 {
            x: self.c * other.x - self.s * other.y,
            y: self.s * other.x + self.c * other.y,
        }
    }
}

// /// Rotate a vector
// pub fn mul_t_vec2(&self, v: Vec2) -> Vec2 {
//     Vec2::new(self.c * v.x + self.s * v.y, -self.s * v.x + self.c * v.y)
// }

impl Default for Rot {
    #[inline(always)]
    fn default() -> Self {
        Self { s: 0.0, c: 1.0 }
    }
}

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
#[derive(Clone, Copy, Debug, Default)]
pub struct Transform {
    pub p: Vec2,
    pub q: Rot,
}

impl Transform {
    /// Set this based on the position and angle.
    #[inline(always)]
    pub fn new(p: Vec2, angle: f32) -> Self {
        Self {
            p,
            q: Rot::new(angle),
        }
    }

    /// Get the angle in radians.
    #[inline]
    pub fn get_angle(&self) -> f32 {
        self.q.get_angle()
    }

    /// Get the inverse of this transform: q' * p' = p * q
    #[inline]
    pub fn get_inverse(&self) -> Self {
        let q_inv = self.q.get_inverse();
        Self {
            p: q_inv * -self.p,
            q: q_inv,
        }
    }

    /// Multiply a vector by this matrix.
    #[inline]
    pub fn mul_vec2(&self, v: Vec2) -> Vec2 {
        self.q * v + self.p
    }

    /// Multiply a vector by the inverse of this matrix.
    #[inline]
    pub fn mul_t_vec2(&self, v: Vec2) -> Vec2 {
        self.q.mul_t_vec2(v - self.p)
    }

    /// Multiply a matrix by this matrix.
    #[inline]
    pub fn mul(&self, t: Self) -> Self {
        Self {
            p: self.mul_vec2(t.p),
            q: self.q * t.q,
        }
    }

    /// Multiply a matrix by the inverse of this matrix.
    #[inline]
    pub fn mul_t(&self, t: Self) -> Self {
        let q_inv = self.q.get_inverse();
        Self {
            p: q_inv * (t.p - self.p),
            q: q_inv * t.q,
        }
    }
}

impl Mul<Vec2> for Transform {
    type Output = Vec2;
    #[inline]
    fn mul(self, other: Vec2) -> Vec2 {
        self.mul_vec2(other)
    }
}

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
#[derive(Clone, Copy, Debug)]
pub struct Sweep {
    /// Local center of mass position
    pub local_center: Vec2,

    /// Center world positions
    pub c0: Vec2,
    pub c: Vec2,

    /// World angles
    pub a0: f32,
    pub a: f32,

    /// Fraction of the current time step in the range [0,1]
    /// c0 and a0 are the positions at alpha0.
    pub alpha0: f32,
}

impl Sweep {
    /// Get the interpolated transform at a specific time.
    /// @param beta is a factor in [0,1], where 0 indicates alpha0.
    fn get_transform(&self, beta: f32) -> Transform {
        let mut transform = Transform::default();
        transform.p = (1.0 - beta) * self.c0 + beta * self.c;
        let angle = (1.0 - beta) * self.a0 + beta * self.a;
        transform.q = Rot::new(angle);

        transform
    }

    /// Advance the sweep forward, yielding a new initial state.
    /// @param alpha the new initial time.
    fn advance(&mut self, alpha: f32) {
        assert!(self.alpha0 < 1.0);
        let beta = (alpha - self.alpha0) / (1.0 - self.alpha0);
        self.c0 += beta * (self.c - self.c0);
        self.a0 += beta * (self.a - self.a0);
        self.alpha0 = alpha;
    }

    /// Normalize the angles.
    fn normalize(&mut self) {
        let two_pi = 2.0 * std::f32::consts::PI;
        let d = two_pi * (self.a0 / two_pi).floor();
        self.a0 -= d;
        self.a -= d;
    }
}

impl Default for Sweep {
    #[inline(always)]
    fn default() -> Self {
        Self {
            local_center: Vec2::default(),
            c0: Vec2::default(),
            c: Vec2::default(),
            a0: 0.0,
            a: 0.0,
            alpha0: 0.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn equality_with_tolerance(m1: Mat2x2, m2: Mat2x2, tolerance: f32) -> bool {
        ((m1.col1.x - m2.col1.x).abs() < tolerance)
            && ((m1.col1.y - m2.col1.y).abs() < tolerance)
            && ((m1.col2.x - m2.col2.x).abs() < tolerance)
            && ((m1.col2.y - m2.col2.y).abs() < tolerance)
    }

    #[test]
    fn test_invert() {
        let m = Mat2x2::new(1.0, 2.0, 3.0, 4.0);
        let result = m.invert();
        let expected = Mat2x2::new(-2.0, 1.0, 3.0 / 2.0, -1.0 / 2.0);
        assert!(equality_with_tolerance(result, expected, 0.0001));
    }
}
