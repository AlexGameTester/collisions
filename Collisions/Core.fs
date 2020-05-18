module CollisionsCore

open SFML.System
open System

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

    //This method is probably not correct 'cause coordinates here have different directions and rotation from usual in maths
    static member rotate angle (v: Vector) =
        let x =
            v.X * Math.Cos angle - v.Y * Math.Sin angle

        let y =
            v.X * Math.Sin angle + v.Y * Math.Cos angle

        Vector(x, y)

    static member perpendicular = Vector.rotate (Math.PI / 2.)
    static member Zero = Vector(0., 0.)
    override this.ToString() = String.Format("Vector of ({0}, {1})", this.X, this.Y)


type PointMass private (pos: Vector, vel: Vector, mass: double, id: Guid) =
    new(pos, vel, mass) as this =
        PointMass(pos, vel, mass, Guid.NewGuid())
        then printfn "Point's id is %A" this.Id

    /// <summary>
    /// Unique id for every points that is not changed in process of movement
    /// </summary>
    member val Id = id
    member val Position = pos
    member val Mass = mass
    member val Velocity = vel
    member this.WithVelocity vel = PointMass(this.Position, vel, this.Mass, this.Id)
    member this.WithPosition pos = PointMass(pos, this.Velocity, this.Mass, this.Id)
    member this.WithOffset offset = this.WithPosition(offset + this.Position)
    static member AreSame (p1: PointMass) (p2: PointMass) = p1.Id.Equals p2.Id
    static member CompareId (id: Guid) (p: PointMass) = id.Equals p.Id


    member this.ApplyForce (force: Vector) (dt: double) =
        force
        * (dt / mass)
        + this.Velocity
        |> this.WithVelocity

    //TODO: Temporary made virtual for tests
    abstract Move: double -> PointMass

    default this.Move dt =
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

type ForceFunction = list<PointMass> -> double -> list<PointMass>

type JointFunction = ForceFunction

[<Struct>]
type State(points: list<PointMass>, framerate: int, forceFunctions: list<ForceFunction>, joints: list<JointFunction>) =
    member _.ForceFunctions = forceFunctions
    member _.Joints = joints
    member private _.dt = 1000. / (double framerate)
    member _.Framerate = framerate
    member _.Points = points
    member private this.WithPoints points = State(points, this.Framerate, this.ForceFunctions, this.Joints)

    member this.GetNext() =
        let move = List.map (PointMass.move this.dt)

        let rec apply dt funcs points =
            match funcs with
            | func :: otherFuncs -> func points dt |> apply dt otherFuncs
            | [] -> points

        let applyForces (state: State) =
            apply state.dt state.ForceFunctions state.Points

        let applyJoints dt joints points = apply dt joints points

        applyForces this
        |> applyJoints this.dt this.Joints
        |> move
        |> this.WithPoints
