

extern crate nalgebra;
extern crate num_traits;


use num_traits::{One, Zero, Inv};
use nalgebra::{Quaternion, Real, Vector3};
//use super::*;
//use crate::utils::todo;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Div, DivAssign,Neg};
use std::cmp::Ordering;

// use std::fmt;

// use crate::ext::QuaternionExt;

#[repr(C)]
#[derive(Copy, Clone, Debug)]
pub struct DualQuaternion<N: Real> {
   re: Quaternion<N>,
   du: Quaternion<N>,
}

impl<N: Real> DualQuaternion<N> {
    #[inline]
    pub fn new(re: Quaternion<N>, du: Quaternion<N>) -> Self {
        Self { re, du }
    }

    #[inline]
    pub fn rotation(self) -> Quaternion<N> {
       self.re
    }

    pub fn translation(self) -> Vector3<N> {
       (self.du * self.re.conjugate()).double().imag()
    }
}

impl<N: Real> From<N> for DualQuaternion<N> {
    fn from(v: N) -> Self {
        Self::new(Quaternion::from_parts(v, Vector3::zero()), 
                  Quaternion::zero())
    }
}

impl<N: Real> From<Quaternion<N>> for DualQuaternion<N> {
    fn from(re: Quaternion<N>) -> Self {
        Self::new(re, Quaternion::zero())
    }
}

impl<N: Real> Zero for DualQuaternion<N> {
   fn zero() -> Self {
       Self::new(Quaternion::zero(), Quaternion::zero())
   }

   fn is_zero(&self) -> bool {
       self.re.is_zero() && self.du.is_zero()
   }
}

impl<N: Real> One for DualQuaternion<N> {
    fn one() -> Self {
       Self::new(Quaternion::one(), Quaternion::zero())
   }
}

impl<N: Real> Default for DualQuaternion<N> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<N: Real> Signed {
    
}

    // /// homogenous coordinates
    // /// 
    // fn homo() {
        
    // }

// impl<N: Real> Signed for DualQuaternion<N> {

// }

impl<N: Real> Inv for DualQuaternion<N> {
    type Output = Self;

    fn inv(self) -> Self::Output {
       self.conjugate() / self.magnitude().abs()
   }
}

impl<N: Real> DualQuaternion<N> {

    /// needs attention
   pub fn conjugate(self) -> Self {
       Self::new(self.re.conjugate(), self.du.conjugate())
   }

   pub fn magnitude(&self) -> N {
       self.re.dot(&self.du)
   }

   pub fn dot(&self, other: &Self) -> N {
       self.re.dot(&other.re)
   }

   pub fn normalize(&self) -> Self {
       *self / self.magnitude().abs()
   }

   pub fn exp(self) -> Self {
       let r = self.re.exp();
       /// what about the order?
       Self::new(r, r * self.du)
   }

   pub fn pow(self, t: N) -> Self {
       unimplemented!()
//        (self.log() * t).exp()
   }

   pub fn square(self) -> Self {
       self * self
   }

//    pub fn scale(self) -> N {
//        N::from_f64(1.0f64) / self.re.magnitude()
//    }

   pub fn ln(self) -> Self {
       Self::new(self.re.ln(), self.du * self.re.try_inverse().unwrap())
   }

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


   pub fn slerp(self, other: Self, t: N) -> Self {
       (other * self.conjugate()).pow(t) * self
   }
}

impl<N: Real> PartialEq for DualQuaternion<N> {
   fn eq(&self, other: &Self) -> bool {
       self.re == other.re && self.du == other.du
   }
}

impl<N: Real> PartialOrd for DualQuaternion<N> {
   fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
       unimplemented!()
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
       Self::new(self.re * other.re.try_inverse().unwrap(),
                (self.du * other.re - self.re * other.du) * (other.re * other.re).try_inverse().unwrap())
   }
}

impl<N: Real> DivAssign for DualQuaternion<N> {
   fn div_assign(&mut self, other: Self) {
       *self = *self / other
   }
}

impl<N: Real> Neg for DualQuaternion<N> {
   type Output = Self;
   fn neg(self) -> Self {
       Self::new(self.re.neg(), self.du.neg())
   }
}

///
/// T operations
///

impl<N: Real> Add<N> for DualQuaternion<N> {
   type Output = Self;
   fn add(self, other: N) -> Self {
       self + Self::from(other)
   }
}

impl<N: Real> AddAssign<N> for DualQuaternion<N> {
   fn add_assign(&mut self, other: N) {
       *self = *self + other
   }
}

impl<N: Real> Sub<N> for DualQuaternion<N> {
   type Output = Self;
   fn sub(self, other: N) -> Self {
       self - Self::from(other)
   }
}

impl<N: Real> SubAssign<N> for DualQuaternion<N> {
   fn sub_assign(&mut self, other: N) {
       *self = *self - other
   }
}

impl<N: Real> Mul<N> for DualQuaternion<N> {
   type Output = Self;
   fn mul(self, other: N) -> Self {
       self * Self::from(other)
   }
}

impl<N: Real> MulAssign<N> for DualQuaternion<N> {
   fn mul_assign(&mut self, other: N) {
       *self = *self * other
   }
}

impl<N: Real> Div<N> for DualQuaternion<N> {
   type Output = Self;
   fn div(self, other: N) -> Self {
       self / Self::from(other)
   }
}


impl<N: Real> DivAssign<N> for DualQuaternion<N> {
   fn div_assign(&mut self, other: N) {
       *self = *self / other
   }
}


impl<N: Real> DualQuaternion<N> {
    // #[inline]
    // fn powi(self, n: i32) -> Self {
    //     let nf = <T as NumCast>::from(n).expect("Invalid value");

    //     Self::new(self.re.powi(n), nf * self.real().powi(n - 1) * self.dual())
    // }

//    #[inline]
//    fn cbrt(self) -> Self {
//        let real = self.re.cbrt();

//        Self::new(real, self.du / (T::from(3).unwrap() * real))
//    }

   #[inline]
   fn hypot(self, other: Self) -> Self {
    //    self.re.powf(n: N)
       unimplemented!()
    //    let real = self.re.hypot(other.re);
    //    Self::new(real, (self.re * other.du + other.re * self.du) / real)
   }

   #[inline]
   fn sin(self) -> Self {
       Self::new(self.re.sin(), self.du * self.re.cos())
   }

   #[inline]
   fn asin(self) -> Self {
       unimplemented!()
    //    let two: N = N::from_f64(2.0f64);
    //    Self::new(self.re.asin(), self.du / (N::one() - self.re.powf(two)).sqrt())
   }

   #[inline]
   fn cos(self) -> Self {
       Self::new(self.re.cos(), self.du.neg() * self.re.sin())
   }

   #[inline]
   fn acos(self) -> Self {
       unimplemented!()
    //    Self::new(self.re.acos(), self.du.neg() / (T::one() - self.re.powi(2)).sqrt())
   }

   #[inline]
   fn tan(self) -> Self {
       let t = self.re.tan();
        unimplemented!()
    //    Self::new(t, self.du * (t * t + N::one()))
   }

   #[inline]
   fn atan(self) -> Self {
       unimplemented!()
    //    Self::new(self.re.atan(), self.du / (self.re.powi(2) + T::one()).sqrt())
   }

   #[inline]
   fn atan2(self, other: Self) -> Self {
       unimplemented!()
    //    Self::new(
        //    self.re.atan2(other.re),
        //    (other.re * self.du - self.re * other.du) / (self.re.powi(2) + other.re.powi(2)),
    //    )
   }

//    #[inline]
//    fn sin_cos(self) -> (Self, Self) {
//        let (s, c) = self.re.sin_cos();

//        let sn = Self::new(s, self.du * c);
//        let cn = Self::new(c, self.du.neg() * s);

//        (sn, cn)
//    }

//    #[inline]
//    fn exp_m1(self) -> Self {
//        Self::new(self.re.exp_m1(), self.du * self.re.exp())
//    }

//    #[inline]
//    fn ln_1p(self) -> Self {
//        Self::new(self.re.ln_1p(), self.du / (self.re + T::one()))
//    }

   #[inline]
   fn sinh(self) -> Self {
       Self::new(self.re.sinh(), self.du * self.re.cosh())
   }

//    #[inline]
//    fn asinh(self) -> Self {
//        Self::new(self.re.asinh(), self.du / (self.re.powi(2) + T::one()).sqrt())
//    }

   #[inline]
   fn cosh(self) -> Self {
       Self::new(self.re.cosh(), self.du * self.re.sinh())
   }

   #[inline]
   fn tanh(self) -> Self {
       let real = self.re.tanh();
        unimplemented!()
    //    Self::new(real, self.du * (T::one() - real.powi(2)))
   }

//    #[inline]
//    fn acosh(self) -> Self {
//        Self::new(
//            self.re.acosh(),
//            self.du / ((self.re + T::one()).sqrt() * (self.re - T::one()).sqrt()),
//        )
//    }

//    #[inline]
//    fn atanh(self) -> Self {
//        Self::new(self.re.atanh(), self.du / (T::one() - self.re.powi(2)))
//    }
}


//extern crate nalgebra;
//use nalgebra::{Quaternion, Real};
//
//use std::fmt::{Debug, Display, Formatter, Result as FmtResult};
//
//struct DualQuaternion<N: Real> {
//    real: Quaternion<N>,
//    dual: Quaternion<N>,
//}
//
//

//


////impl<N: Real> Debug for DualQuaternion<N>  {
////    fn fmt(&self, f: &mut Formatter) -> FmtResult {
////        f.debug_tuple("Dual").field(self.real).field(self.dual).finish()
////    }
////}
//
//
////fn dqt<T: Real>(d: DualQuaternion<N>) {
////    let d :DualQuaternion<N> = DualQuaternioT::one();
////    dump(d)
////}
//
