
extern crate dual_quaternion;
use dual_quaternion::dual_quaternion::DualQuaternion;

extern crate nalgebra;
use nalgebra::Quaternion;

extern crate num_traits;
use num_traits::Inv;

#[macro_use]
extern crate approx;

fn dq() -> DualQuaternion<f64> {
    let re = Quaternion::new(0.1, 0.2, 0.1, 0.2);
    let du = Quaternion::new(0.3, 0.1, 0.1, 0.2);
    DualQuaternion::<f64>::new(re, du)
}

fn test_sin() {
    let dq = dq();
    let r = dq.sin().asin();
    println!("{:?}", r);
}

fn test_sinh() {
    let dq = dq();
    let r = dq.sinh().asinh();
    println!("{:?}", r);
}

fn test_cos() {
    let dq = dq();
    let r = dq.cos().acos();
    println!("{:?}", r);
}


fn test_cosh() {
    let dq = dq();
    let r = dq.cosh().acosh();
    println!("{:?}", r);
}

fn test_tan() {
    let dq = dq();
    let r = dq.tan().atan();
    println!("{:?}", r);
}

fn test_inv() {
    let dq = dq();

    let r = dq.inv().inv();
    println!("{:?}", r);
}

fn test_exp() {
    let dq = dq();
    let r = dq.exp().ln();
    println!("{:?}", r);

}

fn test_qxp() {
    let input = dq();
    let result = input.exp().ln();
    // println!("{}", result);
    
    assert_relative_eq!(input, result);
}



fn main() {
    // let re = Quaternion::new(1.0, 2.0, 3.0, 4.0);
    // println!("{}", re);
    // test_exp();
    // test_exp();
    // let input = dq();
    // println!("{}", input.exp());
    // test_sin();
}