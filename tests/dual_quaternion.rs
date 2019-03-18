
extern crate nalgebra;
use nalgebra::Quaternion;

extern crate dual_quaternion;
use dual_quaternion::dual_quaternion::{DualQuaternion};

#[macro_use]
extern crate approx;

fn dq() -> DualQuaternion<f64> {
    let re = Quaternion::new(0.1, 0.2, 0.1, 0.2);
    let du = Quaternion::new(0.3, 0.1, 0.1, 0.2);
    DualQuaternion::<f64>::new(re, du)
}

#[test]
fn test_sin() {
    let input = dq();
    let output = input.cos().acos();
    assert_relative_eq!(input, output);
}

#[test]
fn test_rotation() {
    
}