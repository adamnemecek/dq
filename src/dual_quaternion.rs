extern crate nalgebra;
extern crate num_traits;

pub use num_traits::{One, Zero, Inv, Pow, Signed, Num};
use nalgebra::{Quaternion, Real, Vector3};
//use super::*;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign,Neg};
use std::cmp::Ordering;
use crate::screw::Screw;

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
        let a = (self.du * self.re.conjugate());
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

//    pub fn scale(self) -> N {
//        N::from_f64(1.0f64) / self.re.magnitude()
//    }

    #[inline]
    pub fn exp(self) -> Self {
        let r = self.re.exp();
        /// what about the order?
        Self::new(r, r * self.du)
    }

    #[inline]
    pub fn ln(self) -> Self {
        Self::new(self.re.ln(), self.du.right_div(&self.re).unwrap())
    }

    #[inline]
    pub fn log(self, base: Self) -> Self {
        self.ln() / base.ln()
    }

//    pub fn sqft(self) -> Self {
//        (self.ln().half().exp()
//     //    unimplemented!()
//    }


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
        self.re == other.re && self.du == other.du
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

    #[inline]
    pub fn hypot(self, other: Self) -> Self {
        // self.to_homogeneous();
    //    self.re.powf(n: N)
           unimplemented!()
    //    let real = self.re.hypot(other.re);
    //    Self::new(real, (self.re * other.du + other.re * self.du) / real)
    }

    /// Sinus.
    /// sin(u, u') = (sin(u), u'*cos(u))
    #[inline]
    pub fn sin(self) -> Self {
        Self::new(self.re.sin(), self.du * self.re.cos())
    }

    /// Arcsinus.
    /// asin(u, u') = (asin(u), u' / sqrt(1 - u^2))
    #[inline]
    pub fn asin(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(self.re.asin(), self.du.right_div(&(one - self.re.squared()).sqrt()).unwrap())
    }

    /// Cosinus.
    /// cos(u, u') = (cos(u), -u'*sin(u))
    #[inline]
    pub fn cos(self) -> Self {
        Self::new(self.re.cos(), (-self.du) * self.re.sin())
    }

    /// Arccosinus.
    /// acos(u, u') = (acos(u), -u' / sqrt(1 - u^2))
    #[inline]
    pub fn acos(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(self.re.acos(), (-self.du).right_div(&(one - self.re.squared()).sqrt()).unwrap())
    }

    /// Tangent.
    /// tan(u, u') = (tan(u), u' * (tan(u)^2 + 1))
    #[inline]
    pub fn tan(self) -> Self {
        let one = Quaternion::<N>::one();
        let t = self.re.tan();
        Self::new(t, self.du * (t * t + one))
    }

    /// Arctangent.
    /// atan(u, u') = (atan(u), u' / sqrt(u^2 + 1))
    #[inline]
    pub fn atan(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(self.re.atan(), self.du.right_div(&(self.re.squared() + one).sqrt()).unwrap())
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
    /// sinh(u, u') = (sinh(u), u' * cosh(u))
    #[inline]
    pub fn sinh(self) -> Self {
        Self::new(self.re.sinh(), self.du * self.re.cosh())
    }

    /// Hyperbolic arcsinus.
    /// asinh(u, u') = (asinh(u), u' / sqrt(u^2 + 1))
    #[inline]
    pub fn asinh(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(self.re.asinh(), self.du.right_div(&(self.re.squared() + one)).unwrap().sqrt())
    }

    /// Hyperbolic cosinus.
    /// cosh(u, u') = (cosh(u), u' * sinh(u))
    #[inline]
    pub fn cosh(self) -> Self {
        Self::new(self.re.cosh(), self.du * self.re.sinh())
    }

    /// Hyperbolic arccosinus.
    /// acosh(u, u') = (acosh(u), u' / (sqrt(u + 1) * sqrt(u - 1)))
    #[inline]
    pub fn acosh(self) -> Self {
        let one = Quaternion::<N>::one();
        Self::new(
            self.re.acosh(),
            self.du.right_div(&((self.re + one).sqrt() * (self.re - one).sqrt())).unwrap()
        )
    }

    /// Hyperbolic tangent.
    /// tanh(u, u') = (u, u' * (1 - u^2))
    #[inline]
    pub fn tanh(self) -> Self {
        let one = Quaternion::<N>::one();
        let real = self.re.tanh();
        Self::new(real, self.du * (one - real.squared()))
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
