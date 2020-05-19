module Joints

open CollisionsCore
open CollisionsCore.CommonFunctional
open System


//TODO: When joint is created, it should change  points' velocities to prevent changes in distance
let getConstDistance (p1: PointMass) (p2: PointMass) =
    let constDistance (id1: Guid) (id2: Guid) =
        fun (state: list<Force * PointMass>) (dt: double) ->
            let (f1, p1) =
                List.find (fun (force, point) -> PointMass.CompareId id1 point) state

            let (f2, p2) =
                List.find (fun (force, point) -> PointMass.CompareId id2 point) state

            let fromFstToSnd =
                (p2.Position - p1.Position) |> Vector.normalized

            let perpendicular = fromFstToSnd |> Vector.perpendicular

            let stateOfOther =
                swap List.filter state
                <| fun (force, point) ->
                    not (PointMass.CompareId id1 point)
                    && not (PointMass.CompareId id2 point)

            let f1OnAxis = Vector.dot fromFstToSnd f1
            let f2OnAxis = Vector.dot fromFstToSnd f2

            let tension = (f1OnAxis - f2OnAxis) * 0.5
            let t1 = fromFstToSnd * tension
            let t2 = fromFstToSnd * (-tension)

            [ (f1 + t1, p1); (f2 + t2, p2) ] @ stateOfOther

    constDistance p1.Id p2.Id


let getConstDistance2 (p1: PointMass) (p2: PointMass) =
    let error = 1.0e-8
    let mutable p1AdditionalVelocity = Vector.Zero
    let mutable p2AdditionalVelocity = Vector.Zero

    let initialDistance =
        p1.Position - p2.Position |> Vector.magnitude

    let constDistance (id1: Guid) (id2: Guid) =
        fun (points: list<PointMass>) (dt: double) ->
            let p1 =
                List.find (PointMass.CompareId id1) points

            let p2 =
                List.find (PointMass.CompareId id2) points

            let fromFstToSnd =
                (p2.Position - p1.Position) |> Vector.normalized

            let perpendicular = fromFstToSnd |> Vector.perpendicular

            let otherPoints =
                swap List.filter points
                <| fun p ->
                    not (PointMass.CompareId id1 p)
                    && not (PointMass.CompareId id2 p)

            let newVelocity =
                let p1ImpulseOnAxis =
                    p1.Velocity * p1.Mass |> Vector.dot fromFstToSnd

                let p2ImpulseOnAxis =
                    p2.Velocity * p2.Mass |> Vector.dot fromFstToSnd

                (p1ImpulseOnAxis + p2ImpulseOnAxis)
                / (p1.Mass + p2.Mass)

            // printfn "New velocity is %f" newVelocity

            let apply =
                fun (p: PointMass) ->
                    let perpendicularVelocity =
                        Vector.dot perpendicular p.Velocity
                        |> (*) perpendicular

                    p.WithVelocity(fromFstToSnd * newVelocity + perpendicularVelocity)

            // TODO: Change this 'cause velocity isn't vanished by previous code in
            //       implulse conservation because of rotation of joint
            //       Не смотря на то что импульс сохраняется, эта скорость не уничтожается взаимно
            //       в манипуляциях с импульсом выше, т.к. связь (стержень) вращается
            let supressDistanceChanges (p1: PointMass, p2: PointMass) =
                let dist =
                    p1.Position - p2.Position |> Vector.magnitude

                let difference = dist - initialDistance
                if Math.Abs difference > error then
                    let additionalVelocityPerMass =
                        difference / (2. * dt * p1.Mass * p2.Mass)

                    p1AdditionalVelocity <- fromFstToSnd * additionalVelocityPerMass * p1.Mass

                    p2AdditionalVelocity <-
                        fromFstToSnd
                        * (-additionalVelocityPerMass)
                        * p2.Mass
                    [ p1.WithVelocity(p1.Velocity + p1AdditionalVelocity)
                      p2.WithVelocity(p2.Velocity + p2AdditionalVelocity) ]
                else
                    [ p1; p2 ]

            let updateVelocities (p1: PointMass) (p2: PointMass) =
                let p1New =
                    if not (p1AdditionalVelocity.Magnitude.Equals 0.0) then
                        let ret =
                            p1.WithVelocity(p1.Velocity - p1AdditionalVelocity)

                        p1AdditionalVelocity <- Vector.Zero
                        ret
                    else
                        p1

                let p2New =
                    if not (p2AdditionalVelocity.Magnitude.Equals 0.0) then
                        let ret =
                            p2.WithVelocity(p2.Velocity - p2AdditionalVelocity)

                        p2AdditionalVelocity <- Vector.Zero
                        ret
                    else
                        p2

                (p1New, p2New)




            (*                 if Math.Abs difference > error then
                    let differencePerMass = difference / (p1.Mass + p2.Mass)
                    [ p1.WithOffset(fromFstToSnd * (differencePerMass * p1.Mass))
                      p2.WithOffset(fromFstToSnd * (-differencePerMass * p2.Mass)) ]
                else
                    [ p1; p2 ] *)



            // supressDistanceChanges (apply p1) (apply p2)
            // [ apply p1; apply p2 ]
            updateVelocities p1 p2
            |> supressDistanceChanges
            |> fun lst ->
                let dist =
                    lst.[0].Position
                    - lst.[1].Position
                    |> Vector.magnitude

                if Math.Abs(initialDistance - dist)
                   / initialDistance > 0.03 then
                    printfn "Distance changed dramatically to %f" dist
                lst
            |> (@) otherPoints

    constDistance p1.Id p2.Id
