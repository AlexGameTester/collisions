module Forces

open CollisionsCore
open CollisionsCore.CommonFunctional



let simpleDown value: ForceFunction =
    let down = Vector(0., 1.)
    fun state dt -> List.map (fun (f, p) -> (f + (down * value), p)) state

let commonBetweenPairs (force: PointMass -> PointMass -> Vector) =
    fun (state: list<Force * PointMass>) (_: double) ->
        let points = List.map snd state
        swap List.map state
        <| fun (forceBefore, point) ->
            List.map (force point) points
            |> List.fold (+) Vector.Zero
            |> (+) forceBefore
            |> swap pack point

(* swap List.map points (fun point ->
                List.map (forceFromFstToSnd point) points
                |> List.fold (+) Vector.Zero
                |> swap point.ApplyForce dt) *)

let gravity =
    let force (toPoint: PointMass) (fromPoint: PointMass) =
        //TODO: Maybe add this check directly to commonBetweenPairs
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
