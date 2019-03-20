extern crate nalgebra;
extern crate num_traits;

use nalgebra::{Real, Vector3};

#[derive(Clone, Copy, Debug)]
pub struct Screw<N: Real> {
    pub direction: Vector3<N>,
    pub moment: Vector3<N>,
    pub angle: N,
    pub pitch: N,
}

impl<N: Real> Screw<N> {
    pub fn new(direction: Vector3<N>, moment: Vector3<N>, angle: N, pitch: N) -> Self {
        Self { direction, moment, angle, pitch }
    }
}
impl<N: Real> Default for Screw<N> {
    fn default() -> Self {
        unimplemented!()
    }
}

// impl<N: Real> From<DualQuaternion<N>> for Screw<N> {
//     fn from(s: DualQuaternion<N>) -> Self {
//         unimplemented!()
//     }
// }