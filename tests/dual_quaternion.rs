
extern crate nalgebra;
use nalgebra::Quaternion;

extern crate dual_quaternion;
use dual_quaternion::dual_quaternion::*;



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


// #[test]
// fn test_pow() {

//     let input = dq();
//     let a = input.exp();
//     let b = input.exp_v2();
    
//     // let a = input * input;
//     // let b = input.pow(2.0);
    
//     assert_relative_eq!(a, b, epsilon = 1.0e-7);
// }



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