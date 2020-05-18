module CollisionTests

open NUnit.Framework
open CollisionsCore

[<SetUp>]
let Setup () = ()

[<Test>]
let Perpendicular_IVector_ShouldReturnMinusJ () =
    let expected = Vector(0., 1.)
    let I = Vector(1., 0.)
    let actual = Vector.perpendicular I
    Assert.AreEqual(expected, actual)
