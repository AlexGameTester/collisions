module Forces

open CollisionsCore
open CollisionsCore.CommonFunctional



let simpleDown value =
    fun (points: list<PointMass>) dt -> List.map (PointMass.applyForce (Vector(0., value)) dt) points

let gravityBetweenTwo (points: list<PointMass>) dt =
    let fst = points.Head
    let snd = points.[1]

    let fromFstToSnd =
        snd.Position - fst.Position |> Vector.normalized

    let forceMagnitude =
        let distSq =
            (fst.Position - snd.Position)
            |> Vector.magnitude
            |> CommonFunctional.swap pown 2

        fst.Mass * snd.Mass / distSq

    [ fst.ApplyForce (fromFstToSnd * forceMagnitude) dt
      snd.ApplyForce (fromFstToSnd * (-forceMagnitude)) dt ]

let commonBetweenPairs (forceFromFstToSnd: PointMass -> PointMass -> Vector) =
    fun (points: list<PointMass>) dt ->
        swap List.map points (fun point ->
            List.map (forceFromFstToSnd point) points
            |> List.fold (+) Vector.Zero
            |> swap point.ApplyForce dt)

let gravity =
    let force (toPoint: PointMass) (fromPoint: PointMass) =
        if (obj.ReferenceEquals(toPoint, fromPoint)) then
            Vector.Zero
        else
            let vectorBetween = fromPoint.Position - toPoint.Position
            let direction = vectorBetween |> Vector.normalized

            let sqrMagnitude =
                vectorBetween |> Vector.magnitude |> swap pown 2

            direction
            * (toPoint.Mass * fromPoint.Mass / sqrMagnitude)

    commonBetweenPairs force

let gravityToCenter center mass =
    let ms = swap List.map

    let force (p: PointMass) =
        let magn = p.Position - center |> Vector.magnitude
        (p.Position - center)
        * (mass * p.Mass / (pown magn 2))

    fun (points: list<PointMass>) dt -> List.map (fun p -> PointMass.applyForce (force p) dt p) points

let collideWithBottom (bottom: double) =
    fun (points: list<PointMass>) dt ->
        let apply (point: PointMass) =
            let positionAfterInterval = (point.Move dt).Position

            if point.Position.Y > bottom then
                Vector(point.Velocity.X, -point.Velocity.Y)
                |> point.WithVelocity
            elif positionAfterInterval.Y > bottom then
                printfn "Before collision point had velocity of (%f, %f)" point.Velocity.X point.Velocity.Y
                let distAfterCollision = positionAfterInterval.Y - bottom
                Vector(point.Velocity.X, -point.Velocity.Y)
                |> point.WithVelocity
                |> fun point ->
                    printfn "After collision point has velocity of (%f, %f)" point.Velocity.X point.Velocity.Y
                    point.WithPosition(Vector(point.Position.X, bottom - distAfterCollision))

            else
                point

        List.map apply points
