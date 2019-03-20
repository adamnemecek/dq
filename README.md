# dq

Implementation of dual quaternions uses [nalgebra](https://github.com/rustsim/nalgebra)  and is heavily inspired by [dual_num](https://github.com/novacrazy/dual_num).


# Resources

The trigonometric implementation differs slightly from what one might find online (e.g. [wikipedia](https://en.wikipedia.org/wiki/Automatic_differentiation#Automatic_differentiation_using_dual_numbers)). Quaternions form a non-commutative algebra and multiplications by `u'` should actually be in the other order. E.g. 
`sin(u, u') = (sin(u), u'*cos(u))` should actually be `sin(u, u') = (sin(u), cos(u) * u')`.



https://github.com/potan/dual.rs/blob/master/src/lib.rs


https://blog.demofox.org/2014/12/30/dual-numbers-automatic-differentiation/

https://idontgetoutmuch.wordpress.com/2013/10/13/backpropogation-is-just-steepest-descent-with-automatic-differentiation-2/

https://github.com/hoechp/ultracomplexmath/blob/954c19190e/src/util/hypercomplex/Dual.java

note that trigonometry diverges at certain places from idontgetoutmuch as quaternions are non-commutative

https://github.com/dpd3788/quatlib/blob/7b42455542edbc3b62df3ee20eb8c465d81f6ac0/cpp/include/quaternion.h

## TODO
* conjugates
* homogenous matrix conversion

* why doesn't a.pow(4.0).pow(1.0/4.0) == a? should it
* randomization
* pow -> slerp

