
extern crate nalgebra;
use nalgebra::Quaternion;

extern crate dual_quaternion;
use dual_quaternion::dual_quaternion::{DualQuaternion};

#[macro_use]
extern crate approx;

fn dq() -> DualQuaternion<f64> {
    let re = Quaternion::new(0.1, 0.2, 0.1, 0.2);
    let du = Quaternion::new(0.3, 0.1, 0.1, 0.2);
    DualQuaternion::new(re, du)
}



#[test]
fn test_exp() {
    let input = dq();
    let output = input.exp().ln();
    assert_relative_eq!(input, output);
}

/// Trigonometry

#[test]
fn test_cos() {
    let input = dq();
    let output = input.cos().acos();
    assert_relative_eq!(input, output, epsilon = 1.0e-7);
}

// #[test]
// fn test_sin() {
//     let input = dq();
//     let output = input.cos().acos();
//     assert_relative_eq!(input, output);
// }


// #[test]
// fn test_cosh() {
//     let input = dq();
//     let output = input.cosh().acosh();
//     assert_relative_eq!(input, output, epsilon = 1.0e-7);
// }

// #[test]
// fn test_tan() {
//     let input = dq();
//     let output = input.tan().atan();
//     assert_relative_eq!(input, output, epsilon = 1.0e-7);
// }

// #[test]
// fn test_qtan() {
//     let input = dq().re;
//     let output = input.tan().atan();
//     assert_relative_eq!(input, output, epsilon = 1.0e-7);
// }



// #[test]
// fn test_rotation() {
    
// }