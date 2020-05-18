module Joints

open CollisionsCore
open CollisionsCore.CommonFunctional
open System

let getConstDistance (p1: PointMass) (p2: PointMass) =
    let constDistance (id1: Guid) (id2: Guid) =
        fun (points: list<PointMass>) (_: double) ->
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

            let p1ImpulseOnAxis =
                p1.Velocity * p1.Mass |> Vector.dot fromFstToSnd

            let p2ImpulseOnAxis =
                p2.Velocity * p2.Mass |> Vector.dot fromFstToSnd

            let newVelocity =
                p1ImpulseOnAxis
                + p2ImpulseOnAxis
                / (p1.Mass + p2.Mass)

            let apply =
                fun (p: PointMass) ->
                    let perpendicularVelocity =
                        Vector.dot perpendicular p.Velocity
                        |> (*) perpendicular

                    p.WithVelocity(fromFstToSnd * newVelocity + perpendicularVelocity)

            [ apply p1; apply p2 ] @ otherPoints

    constDistance p1.Id p2.Id
