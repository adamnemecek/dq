
extern crate nalgebra;
use nalgebra::Quaternion;

mod dual_quaternion;
use dual_quaternion::DualQuaternion;


#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
    #[test]
    fn test_sin() {
        let re = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let du = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let dq = DualQuaternion::<f64>::new(re, du);

    }
}
