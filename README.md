# Index-range subscript operators

This is a prototype implementation of subscripting for any kind of contiguous 
or random-access range. The idea is to slice out a sub-range with the most 
efficient type (`empty_view`, `span` of static or dynamic extent, `subrange`). 
The subscript operator requires two arguments: first and last/size.

In order to return `span`s of static extent the size must be a constant 
expression when calling the subscript operator. Therefore the indexes can be 
passed as `std::integral_constant` (or any other type that uses the same 
interface). To simplify passing objects of type `integral_constant`, the 
template `Const<Value>` can be used.

If the compiler allows non-member `operator[]` overloads 
(`__GXX_NONMEMBER_SUBSCRIPT__` is pre-defined), then `operator[]` is 
overloaded. Otherwise the function `operator_sub` is used.

## `range[a, b]`

* `a >= 0`: offset from `begin` (`0` is `begin`)
* `a <  0`: offset from `end` (inclusive; `-1` is last element before `end`)
* `b >= 0`: size of resulting range/`span`
* `b <  0`: offset from `end` (inclusive)

## Examples

* `[ 0,  2]`: elements `[0]` and `[1]`
* `[ 4,  3]`: elements `[4]`, `[5]`, and `[6]`
* `[-1, -1]`: the last element
* `[-1,  1]`: the last element
* `[-4, -2]`: the three elements `[-4]`, `[-3]`, and `[-2]` (where `[-1]` is the last element)
* `[-1, -2]`: empty
* `[-1, -3]`: ill-formed if both indexes `constexpr` (negative size requested)
* `[-1,  2]`: ill-formed if both indexes `constexpr` (out of bounds)
* `[ 0, -2]`: all elements but the last
* `[ 1,  0]`: empty
* `[-1,  0]`: empty
