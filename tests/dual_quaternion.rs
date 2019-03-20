
extern crate nalgebra;
use nalgebra::Quaternion;

extern crate dq;
use dq::dual_quaternion::*;

#[macro_use]
extern crate approx;

fn dq() -> DualQuaternion<f64> {
    let re = Quaternion::new(0.1, 0.2, 0.1, 0.2);
    let du = Quaternion::new(0.3, 0.1, 0.1, 0.2);
    DualQuaternion::new(re, du)
}

fn dq2() -> DualQuaternion<f64> {
    let re = Quaternion::new(0.2, 0.3, 0.4, 0.1);
    let du = Quaternion::new(0.2, 0.1, 0.3, 0.4);
    DualQuaternion::new(re, du)
}

#[test]
fn test_exp() {
    let input = dq();
    let result = input.exp().ln();
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

#[test]
fn test_pow() {
    let input = dq();
    let e = 1.5f64;
    let result = input.pow(e).pow(1.0/e);
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

#[test]
fn test_pow2() {
    let input = dq();
    let e = 9.0f64;
    let result = input.pow(1.0/e).pow(e);
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

#[test]
fn test_conjugate() {
    let a = dq();
    let b = dq2();

    let left = (a * b).conjugate();
    let right = b.conjugate() * a.conjugate();
    assert_relative_eq!(left, right, epsilon = 1.0e-7);
}


// #[test]
// fn test_slerp() {
//     let a = dq();
//     let b = dq2();

//     let c = a.slerp(&b, 1.5f64);

//     assert_relative_eq!(left, right, epsilon = 1.0e-7);
// }

#[test]
fn test_inv() {
    let input = dq();

    let result = input.inv().inv();
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

/// Trigonometry

#[test]
fn test_cos() {
    let input = dq();
    let result = input.cos().acos();
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

#[test]
fn test_sin() {
    let input = dq();
    let result = input.sin().asin();
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

#[test]
fn test_tan() {
    let input = dq();
    let result = input.tan().atan();
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

#[test]
fn test_cosh() {
    let input = dq();
    let result = input.cosh().acosh();
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

#[test]
fn test_sinh() {
    let input = dq();
    let result = input.sinh().asinh();
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}

#[test]
fn test_tanh() {
    let input = dq();
    let result = input.tanh().atanh();
    assert_relative_eq!(input, result, epsilon = 1.0e-7);
}


// #[test]
// fn test_rotation() {
    
// }