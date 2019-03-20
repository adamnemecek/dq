
extern crate dq;
use dq::dual_quaternion::DualQuaternion;

extern crate nalgebra;
use nalgebra::Quaternion;

// extern crate num_traits;
// use num_traits::Inv;

// #[macro_use]
// extern crate approx;

// fn dq() -> DualQuaternion<f64> {
//     let re = Quaternion::new(0.1, 0.2, 0.1, 0.2);
//     let du = Quaternion::new(0.3, 0.1, 0.1, 0.2);
//     DualQuaternion::<f64>::new(re, du)
// }

// fn dq2() -> DualQuaternion<f64> {
//     let re = Quaternion::new(0.2, 0.3, 0.4, 0.1);
//     let du = Quaternion::new(0.2, 0.1, 0.3, 0.4);
//     DualQuaternion::new(re, du)
// }

// fn test_inv() {
//     let dq = dq();

//     let r = dq.inv().inv();
//     println!("{:?}", r);
// }

// fn test_slerp() {
//     let a = dq();
//     let b = dq2();
//     let result = a.slerp(b, 0.5);
//     // assert_relative_eq!(input, result, epsilon = 1.0e-7);
//     println!("{}", result);
// }


// fn test_qxp() {
//     let input = dq();
//     let result = input.exp().ln();
//     // println!("{}", result);
    
//     assert_relative_eq!(input, result);
// }



// fn main() {
//     // let re = Quaternion::new(1.0, 2.0, 3.0, 4.0);
//     // println!("{}", re);
//     // test_exp();
//     // test_exp();
//     // let input = dq();
//     // println!("{}", input.exp());
//     // test_sin();
//     test_slerp();
// }




fn dq() -> DualQuaternion<f64> {
    let re = Quaternion::new(0.1, 0.2, 0.1, 0.2);
    let du = Quaternion::new(0.3, 0.1, 0.1, 0.2);
    DualQuaternion::new(re, du)
}


fn main() {
    let input = dq();
    // println!("{}", input.pow(1.5).pow(1.0/1.5));
}


