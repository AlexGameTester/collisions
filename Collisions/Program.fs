﻿module Collisions

open SFML.Graphics
open SFML.Window
open System.Threading.Tasks
open System.Threading

open Core.CommonFunctional
open Core


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
