module Collisions

open SFML.Graphics
open SFML.Window
open System.Threading.Tasks
open System.Threading

open CollisionsCore.CommonFunctional
open CollisionsCore


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

let startWindow inNewThread =
    let inititalState =
        let points =
            [ PointMass(Vector(300., 300.), Vector(0.000, 0.000), 1.0e12)
              PointMass(Vector(300., 600.), Vector(0.05, 0.000), 1.)
              PointMass(Vector(300., 450.), Vector(0.04, 0.000), 1.) ]

        State
            (points,
             60,
             [ Forces.simpleDown 0.000007 ],
             [ Joints.getConstDistance points.[0] points.[1]
               Joints.getConstDistance points.[1] points.[2] ])

    printfn "Creating window"
    new RenderWindow(VideoMode(800u, 600u), "Window")
    |> fun w ->
        //TODO: How exactly does this method work?
        w.SetFramerateLimit 60u
        (*         w.KeyPressed.Add
        //TODO: Event listener is not called for some reason
        <| fun kea ->
            printfn "Some key pressed"
            if kea.Code = Keyboard.Key.Escape then
                printfn "X key released"
                w.Close()
                startWindow ()
            else
                () *)
        w
    |> startDrawing inititalState
    |> fun drawFunction ->
        if inNewThread then
            Task.Factory.StartNew drawFunction
            |> fun task -> task.Wait()
        else
            drawFunction ()

[<EntryPoint>]
let main _ =
    startWindow false
    printfn "Hello World from F#!"
    0 // return an integer exit code
