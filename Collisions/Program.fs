module Collisions

open SFML.Graphics
open SFML.System
open SFML.Window
open System.Threading.Tasks
open System.Threading

module CommonFunctional =
    /// <summary>
    /// Transforms carried function a->b->c to b->a->c (changes order of arguments)
    /// </summary>
    /// <param name="f">Function to transform</param>
    let swap f = fun x y -> f y x
    ///<summary>
    /// Transforms function of touple of 2 arguments into a carried function
    /// </summary>
    let (<*) f x y = f (x, y)
    ///<summary>
    /// Transforms function of touple of 3 arguments into a carried function
    /// </summary>
    let (<**) f x y z = f (x, y, z)

open CommonFunctional
open System

[<Struct>]
type Vector(x: double, y: double) =
    member _.X = x
    member _.Y = y
    member this.Magnitude = sqrt (pown this.X 2 + pown this.Y 2)
    member this.Normalized = Vector(this.X / this.Magnitude, this.Y / this.Magnitude)
    member this.Dot(other: Vector) = this.X * other.X + this.Y * other.Y
    static member (+)(v1: Vector, v2: Vector) = Vector(v1.X + v2.X, v1.Y + v2.Y)
    static member (*)(v: Vector, num: double) = Vector(v.X * num, v.Y * num)
    static member (-)(v1: Vector, v2: Vector) = v1 + (v2 * (-1.))
    static member dot (v1: Vector) (v2: Vector) = v1.Dot v2
    static member magnitude(v: Vector) = v.Magnitude
    static member toV2f(v: Vector) = Vector2f(float32 v.X, float32 v.Y)
    static member normalized(v: Vector) = v.Normalized
    static member Zero = Vector(0., 0.)



type PointMass private (pos: Vector, vel: Vector, mass: double, id: Guid) =
    /// <summary>
    /// Unique id for every points that is not changed in process
    /// </summary>
    new(pos, vel, mass) as this =
        PointMass(pos, vel, mass, Guid.NewGuid())
        then printfn "Point's id is %A" this.Id

    member val private Id = id
    member val Position = pos
    member val Mass = mass
    member val Velocity = vel
    member this.WithVelocity vel = PointMass(this.Position, vel, this.Mass, this.Id)
    member this.WithPosition pos = PointMass(pos, this.Velocity, this.Mass, this.Id)
    static member areSame (p1: PointMass) (p2: PointMass) = p1.Id.Equals p2.Id


    member this.ApplyForce (force: Vector) (dt: double) =
        force
        * (dt / mass)
        + this.Velocity
        |> this.WithVelocity

    member this.Move dt =
        this.Position
        + (this.Velocity * dt)
        |> this.WithPosition

    static member applyForce force dt (pm: PointMass) = pm.ApplyForce force dt
    static member move dt (pm: PointMass) = pm.Move dt

(*     interface IDrawable with
        member this.Draw(window) =
            let point = Vertex(Vector.toV2f this.Position, Color.White)
            let va = new VertexArray()
            va.Append point
            window.Draw va *)

module Forces =
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

    let commonBetweenTwoObjects (forceFromFstToSnd: PointMass -> PointMass -> Vector) =
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

        commonBetweenTwoObjects force

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
                if point.Position.Y > bottom then
                    Vector(point.Velocity.X, -point.Velocity.Y)
                    |> point.WithVelocity
                else
                    point

            List.map apply points







[<StructAttribute>]
type State(points: list<PointMass>, framerate: int, forceFunctions: list<list<PointMass> -> double -> list<PointMass>>) =
    member _.ForceFunctions = forceFunctions
    member private _.dt = 1000. / (double framerate)
    member _.Framerate = framerate
    member _.Points = points
    member private this.WithPoints points = State(points, this.Framerate, this.ForceFunctions)

    member this.GetNext() =
        let move = List.map (PointMass.move this.dt)

        let applyForces (state: State) =
            let rec apply dt forces points =
                match forces with
                | head :: fs -> head points dt |> apply dt fs
                | [] -> points

            apply state.dt state.ForceFunctions state.Points

        applyForces this |> move |> this.WithPoints

let getNext (state: State) = state.GetNext()

let draw (state: State) (window: RenderWindow) =
    let va = new VertexArray()
    // window.Clear()
    for p in state.Points do
        va.Append(Vertex(Vector.toV2f p.Position, Color.White))

    window.Draw va
    window.Display()

let startDrawing (inititalState: State) window =
    let framerate = inititalState.Framerate

    let drawNext state =
        let nextState = getNext state
        draw nextState window
        nextState

    let rec drawOnFrame state =
        let now = System.DateTime.Now
        let nextState = drawNext state
        let delta = System.DateTime.Now - now
        max 0 (1000 / framerate - delta.Milliseconds)
        |> Thread.Sleep
        if window.IsOpen then drawOnFrame nextState else ()

    fun () -> drawOnFrame inititalState

let rec startWindow () =
    let inititalState =
        State
            (*             ([ PointMass(Vector(300., 200.), Vector(0.03, 0.03), 1.)
               PointMass(Vector(200., 200.), Vector(-0.02, -0.03), 1.) ], *)
            ([ PointMass(Vector(100., 200.), Vector(0.02, 0.00), 1.) ],
             90,
             [ Forces.collideWithBottom 500.
               Forces.simpleDown 0.00005 ])

    printfn "Creating window"
    new RenderWindow(VideoMode(800u, 600u), "Window")
    |> fun w ->
        //TODO: How exactly does this method work?
        w.SetFramerateLimit 60u
        w.KeyPressed.Add
        //TODO: Event listener is not called for some reason
        <| fun kea ->
            printfn "Some key pressed"
            if kea.Code = Keyboard.Key.Escape then
                printfn "X key released"
                w.Close()
                startWindow ()
            else
                ()
        w
    |> startDrawing inititalState
    |> Task.Factory.StartNew
    |> fun task -> task.Wait()

[<EntryPoint>]
let main _ =
    startWindow ()
    printfn "Hello World from F#!"
    0 // return an integer exit code
