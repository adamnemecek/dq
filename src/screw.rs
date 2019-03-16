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

impl<N: Real> Default for Screw<N> {
    fn default() -> Self {
        unimplemented!()
    }
}