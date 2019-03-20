extern crate nalgebra;
extern crate num_traits;
extern crate approx;

use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign,Neg};
use std::cmp::Ordering;

pub use num_traits::{One, Zero, Inv, Pow, Signed, Num};
use nalgebra::{Quaternion, Real, Vector3};

use crate::screw::Screw;

use approx::{RelativeEq, AbsDiffEq};

#[repr(C)]
#[derive(Copy, Clone)]
pub struct DualQuaternion<N: Real> {
   pub re: Quaternion<N>,
   pub du: Quaternion<N>,
}

impl<N: Real> DualQuaternion<N> {
    #[inline]
    pub fn new(re: Quaternion<N>, du: Quaternion<N>) -> Self {
        Self { re, du }
    }

    /// New dual quaternion from rotation and translation.
    #[inline]
    pub fn from_rot(rot: Quaternion<N>, t: Vector3<N>) -> Self {
        Self::new(rot, Quaternion::<N>::from_imag(t).half() * rot)
    }

    #[inline]
    pub fn rotation(self) -> Quaternion<N> {
       self.re
    }

    #[inline]
    pub fn translation(self) -> Vector3<N> {
        let a = self.du * self.re.conjugate();
        (a + a).imag()
    }

    /// needs attention
    #[inline]
    pub fn conjugate(self) -> Self {
        Self::new(self.re.conjugate(), self.du.conjugate())
    }

    #[inline]
    pub fn quat_conjugate(&self) -> Self {
        Self::new(self.re.conjugate(), self.du.conjugate())
    }

    #[inline]
    pub fn dual_conjugate(&self) -> Self {
        Self::new(self.re, -self.du)
    }

    #[inline]
    pub fn magnitude(&self) -> N {
        self.re.dot(&self.du)
    }

    #[inline]
    pub fn dot(&self, other: &Self) -> N {
        self.re.dot(&other.re)
    }

    #[inline]
    pub fn normalize(&self) -> Self {
        *self / self.magnitude().abs()
    }

    #[inline]
    pub fn squared(self) -> Self {
        self * self
    }

   pub fn scale(self) -> N {
       N::one() / self.re.norm_squared()
   }

    #[inline]
    pub fn exp(self) -> Self {
        let r = self.re.exp();
        /// what about the order?
        Self::new(r, r * self.du)
    }

    #[inline]
    pub fn ln(self) -> Self {
        let du = (self.re.conjugate() * self.du) * self.scale();
        Self::new(self.re.ln(), du)
    }

    #[inline]
    pub fn log(self, base: Self) -> Self {
        self.ln() / base.ln()
    }

    // pub fn sqrt(self) -> Self {
    //     (self.ln().half().exp()
    // }

//    pub fn lerp(self, other: Self, t: T) -> Self {
//        todo()
//    }

    #[inline]
    pub fn slerp(self, other: Self, t: N) -> Self {
        (other * self.conjugate()).pow(t) * self
    }
}

impl<N: Real> Pow<N> for DualQuaternion<N> {
    type Output = Self;

    #[inline]
    fn pow(self, t: N) -> Self::Output {
        (self.ln() * t).exp()
    }
}

impl<N: Real> From<N> for DualQuaternion<N> {
    #[inline]
    fn from(v: N) -> Self {
        Self::new(Quaternion::from_real(v),
                  Quaternion::zero())
    }
}

impl<N: Real> From<Quaternion<N>> for DualQuaternion<N> {
    #[inline]
    fn from(re: Quaternion<N>) -> Self {
        Self::new(re, Quaternion::zero())
    }
}

impl<N: Real> From<DualQuaternion<N>> for Screw<N> {
    fn from(s: DualQuaternion<N>) -> Self {
        unimplemented!()
    }
}

impl<N: Real> Zero for DualQuaternion<N> {
    #[inline]
    fn zero() -> Self {
        Self::new(Quaternion::zero(), Quaternion::zero())
    }

    #[inline]
    fn is_zero(&self) -> bool {
        self.re.is_zero() && self.du.is_zero()
    }
}

impl<N: Real> One for DualQuaternion<N> {
    #[inline]
    fn one() -> Self {
        Self::new(Quaternion::one(), Quaternion::zero())
    }
}

impl<N: Real> Default for DualQuaternion<N> {
    #[inline]
    fn default() -> Self {
        Self::zero()
    }
}

// impl<N: Real> Signed {

// }

    // /// homogenous coordinates
    // /// 
    // fn homo() {
        
    // }

// use std::hash::{Hash, Hasher};

// impl<N: Real> Hash for DualQuaternion<N> {
//     fn hash<H: Hasher>(&self, state: &mut H) {
//         self.re.hash(state);
//         self.du.hash(state);
//     }
// }


impl<N: Real> Inv for DualQuaternion<N> {
    type Output = Self;

    #[inline]
    fn inv(self) -> Self::Output {
        self.conjugate() / self.magnitude()
    }
}

impl<N: Real> PartialEq for DualQuaternion<N> {
    fn eq(&self, other: &Self) -> bool {
        self.re.eq(&other.re) && self.du.eq(&other.du)
    }
}

impl<N: Real + AbsDiffEq<Epsilon = N>> AbsDiffEq for DualQuaternion<N> {
    type Epsilon = N;

    #[inline]
    fn default_epsilon() -> Self::Epsilon {
        Quaternion::default_epsilon()
    }

    #[inline]
    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.re.abs_diff_eq(&other.re, epsilon) && self.du.abs_diff_eq(&other.du, epsilon)
    }
}

impl<N: Real + RelativeEq<Epsilon = N>> RelativeEq for DualQuaternion<N> {

    #[inline]
    fn default_max_relative() -> Self::Epsilon {
        Quaternion::default_max_relative()
    }

    #[inline]
    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool
    {
        self.re.relative_eq(&other.re, epsilon, max_relative) &&
        self.du.relative_eq(&other.du, epsilon, max_relative)
    }
}

impl<N: Real> PartialOrd for DualQuaternion<N> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.re.magnitude().partial_cmp(&other.magnitude())
    }
}

impl<N: Real> Add for DualQuaternion<N> {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self::new(self.re + other.re, self.du + other.du)
    }
}
impl<N: Real> AddAssign for DualQuaternion<N> {
    fn add_assign(&mut self, other: Self) {
        *self = *self + other;
    }
}

impl<N: Real> Sub for DualQuaternion<N> {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self::new(self.re - other.re, self.du - other.du)
    }
}

impl<N: Real> SubAssign for DualQuaternion<N> {
    fn sub_assign(&mut self, other: Self) {
        *self = *self - other;
    }
}

impl<N: Real> Mul for DualQuaternion<N> {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        Self::new(self.re * other.re, 
                  self.re * other.du + self.du * other.re)
    }
}

impl<N: Real> MulAssign for DualQuaternion<N> {
    fn mul_assign(&mut self, other: Self) {
        *self = *self * other;
    }
}

impl<N: Real> Div<Self> for DualQuaternion<N> {
    type Output = Self;
    fn div(self, other: Self) -> Self {
        Self::new(self.re.right_div(&other.re).unwrap(),
                 (self.du * other.re - self.re * other.du) * (other.re * other.re).inv().unwrap())
    }
}

impl<N: Real> DivAssign for DualQuaternion<N> {
    fn div_assign(&mut self, other: Self) {
        *self = *self / other
    }
}

impl<N: Real> Neg for DualQuaternion<N> {
    type Output = Self;
    
    #[inline]
    fn neg(self) -> Self {
        Self::new(self.re.neg(), self.du.neg())
    }
}

///
/// T operations
///

impl<N: Real> Add<N> for DualQuaternion<N> {
    type Output = Self;

    #[inline]
    fn add(self, other: N) -> Self {
        self + Self::from(other)
    }
}

impl<N: Real> AddAssign<N> for DualQuaternion<N> {
    #[inline]

    fn add_assign(&mut self, other: N) {
        *self = *self + other
    }
}

impl<N: Real> Sub<N> for DualQuaternion<N> {
   type Output = Self;

    #[inline]
    fn sub(self, other: N) -> Self {
        self - Self::from(other)
    }
}

impl<N: Real> SubAssign<N> for DualQuaternion<N> {
    #[inline]
    fn sub_assign(&mut self, other: N) {
        *self = *self - other
    }
}

impl<N: Real> Mul<N> for DualQuaternion<N> {
    type Output = Self;

    #[inline]
    fn mul(self, other: N) -> Self {
        self * Self::from(other)
    }
}

impl<N: Real> MulAssign<N> for DualQuaternion<N> {
    #[inline]
    fn mul_assign(&mut self, other: N) {
        *self = *self * other
    }
}

impl<N: Real> Div<N> for DualQuaternion<N> {
    type Output = Self;

    #[inline]
    fn div(self, other: N) -> Self {
        self / Self::from(other)
    }
}

impl<N: Real> DivAssign<N> for DualQuaternion<N> {
    #[inline]
    fn div_assign(&mut self, other: N) {
        *self = *self / other
    }
}

impl<N: Real> DualQuaternion<N> {
    // #[inline]
    // fn powf(self, n: i32) -> Self {
    //     let nf = <T as NumCast>::from(n).expect("Invalid value");

    //     Self::new(self.re.powf(n), nf * self.real().powf(n - 1) * self.dual())
    // }

//    #[inline]
//    fn cbrt(self) -> Self {
//        let real = self.re.cbrt();

//        Self::new(real, self.du / (N::from(3).unwrap() * real))
//    }

    // #[inline]
    // pub fn hypot(self, other: Self) -> Self {
    //     // self.to_homogeneous();
    // //    self.re.powf(n: N)
    //        unimplemented!()
    // //    let real = self.re.hypot(other.re);
    // //    Self::new(real, (self.re * other.du + other.re * self.du) / real)
    // }

    /// Sinus.
    /// sin(u, u') = (sin(u), cos(u) * u')
    #[inline]
    pub fn sin(self) -> Self {
        Self::new(self.re.sin(), self.re.cos() * self.du)
    }

    /// Arcsinus.
    /// asin(u, u') = (asin(u), u' / sqrt(1 - u^2))
    #[inline]
    pub fn asin(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(self.re.asin(), self.du.right_div(&(one - self.re.squared()).sqrt()).unwrap())
    }

    /// Cosinus.
    /// cos(u, u') = (cos(u), -sin(u) * u')
    #[inline]
    pub fn cos(self) -> Self {
        Self::new(self.re.cos(), (-self.re.sin() * self.du))
    }

    /// Arccosinus.
    /// acos(u, u') = (acos(u), -u' / sqrt(1 - u^2))
    #[inline]
    pub fn acos(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(self.re.acos(), (-self.du).right_div(&(one - self.re.squared()).sqrt()).unwrap())
    }

    /// Tangent.
    /// tan(u, u') = (tan(u), (tan(u)^2 + 1) * u')
    #[inline]
    pub fn tan(self) -> Self {
        let one = Quaternion::<N>::one();
        let t = self.re.tan();
        Self::new(t,(t * t + one) *  self.du)
    }

    /// Arctangent.
    /// atan(u, u') = (atan(u), u' / sqrt(u^2 + 1))
    #[inline]
    pub fn atan(self) -> Self {
        /// todo should re^2 + 1 be sqrt or not?
        let one = Quaternion::<N>::one();
        Self::new(self.re.atan(), self.du.right_div(&(self.re.squared() + one)).unwrap())
    }

//    #[inline]
//    pub fn atan2(self, other: Self) -> Self {
//         Self::new(
//             self.re.atan2(other.re),
//             div(other.re * self.du - self.re * other.du, (self.re.squared() + other.re.squared()),
//         )
//     //    Self::new(
//         //    self.re.atan2(other.re),
//         //    (other.re * self.du - self.re * other.du) / (self.re.squared() + other.re.squared()),
//     //    )
//    }

//    #[inline]
//    fn exp_m1(self) -> Self {
//        Self::new(self.re.exp_m1(), self.du * self.re.exp())
//    }

//    #[inline]
//    fn ln_1p(self) -> Self {
//        Self::new(self.re.ln_1p(), self.du / (self.re + N::one()))
//    }

    /// Hyperbolic sinus.
    /// sinh(u, u') = (sinh(u), cosh(u) * u')
    #[inline]
    pub fn sinh(self) -> Self {
        Self::new(self.re.sinh(), self.re.cosh()* self.du)
    }

    /// Hyperbolic arcsinus.
    /// asinh(u, u') = (asinh(u), u' / sqrt(u^2 + 1))
    #[inline]
    pub fn asinh(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(self.re.asinh(), self.du.right_div(&(self.re.squared() + one).sqrt()).unwrap())
    }

    /// Hyperbolic cosinus.
    /// cosh(u, u') = (cosh(u), u' * sinh(u))
    #[inline]
    pub fn cosh(self) -> Self {
        Self::new(self.re.cosh(), self.re.sinh() * self.du)
    }

    /// Hyperbolic arccosinus.
    /// acosh(u, u') = (acosh(u), u' / (sqrt(u + 1) * sqrt(u - 1)))
    #[inline]
    pub fn acosh(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(
            self.re.acosh(),
            // self.du.right_div(&((self.re + one).sqrt() * (self.re - one).sqrt())).unwrap()
            self.du.right_div(&(self.re.squared() - one).sqrt()).unwrap()
        )
    }

    /// Hyperbolic tangent.
    /// tanh(u, u') = (u, u' * (1 - u^2))
    #[inline]
    pub fn tanh(self) -> Self {
        let one = Quaternion::<N>::one();
        let real = self.re.tanh();
        Self::new(real, (one - real.squared()) * self.du)
    }

    /// Hyperbolic arctangent.
    /// atanh(u, u') = (atanh(u), u' / (1 - u^2))
    #[inline]
    pub fn atanh(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(self.re.atanh(), self.du.right_div(&(one - self.re.squared())).unwrap())
    }
}

// impl<N: Real> Signed for DualQuaternion<N> {
//     fn abs(&self)-> Self {
//         unimplemented!()
//     }

//     fn abs_sub(&self, other: &Self) -> Self {
//         unimplemented!()
//     }

//     fn signum(&self) -> Self {
//         unimplemented!()
//     }

//     fn is_positive(&self) -> bool {
//         unimplemented!()
//     }

//     fn is_negative(&self) -> bool {
//         unimplemented!()
//     }
// }

impl<N: Real> std::fmt::Display for DualQuaternion<N>  {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "DualQuaternion(re: {}, du: {})", self.re, self.du)
    }
}

impl<N: Real> std::fmt::Debug for DualQuaternion<N>  {
    //
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "DualQuaternion(re: {}, du: {})", self.re, self.du)
    }
}
