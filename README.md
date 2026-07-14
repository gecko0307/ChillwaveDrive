# Chillwave Drive
Work-in-progress simcade racing game.

## About the Project

Chillwave Drive is written in [D language](https://dlang.org) using [Dagon Engine](https://github.com/gecko0307/dagon) and [Newton Game Dynamics](https://github.com/madeapps/newton-dynamics).

The game implements its own raycast car simulator, including the famous Pacejka '94 tyre model. The goal is to create a fun and challenging car game that is not so frustratingly hard to play without a driving wheel. The simulator tries to stay both realistic and easy to control with a gamepad or even just a keyboard.

AI-controlled cars are supported. Fairly accurate and stable path following algorithm is implemented, based on the Pure Pursuit method, as well as basic recovery mode.

The project is still in alpha stage. Car and track selection, game settings and other parts of UI are not implemented yet. Core gameplay and the racing logic are mostly done.

[![Screenshot1](https://blog.pixperfect.online/wp-content/uploads/2026/07/chillwave-drive-new-cars-4.jpg)](https://blog.pixperfect.online/wp-content/uploads/2026/07/chillwave-drive-new-cars-4.jpg)

## Controls

- Enter/Start - start the race
- W key / Up arrow key / Right trigger / A button - accelerate
- S key / Down arrow key / Left trigger / B button  - brake/reverse
- A key / Left arrow key / Left stick -X / D-pad left - steer left
- D key / Right arrow key / Left stick +X / D-pad right - steer right
- L key / Left shoulder - toggle headlights
- M key - toggle background music
- Escape key/Start/Back - pause/resume
- LMB + mouse - rotate the camera (only when paused)
- Right stick - rotate the camera
- F4 key - take a screenshot
- F5 key - show debug info.

Xbox controller is recommended. Steering wheels are not supported yet.

In the pause mode it is possible to customize car's appearance using the "Car settings" window.

## Download

Builds are no longer uploaded to GitHub releases. You can download the game on GameJolt page: [https://gamejolt.com/games/chillwave-drive/1030266](https://gamejolt.com/games/chillwave-drive/1030266). Look for "Chillwave Drive Rolling Alpha" package.

## Build from source

Chillwave Drive usually depends on the most recent features of Dagon, so it uses the engine as a local dependency instead of a numbered release. Download a source tarball of Dagon and copy the files to `dagon` directory, then run `dub build  --build=release-nobounds`.

## TODO List

- [x] Vehicle physics
- [x] Sounds
- [x] Opponents AI
- [x] Road/ground material detection
- [x] Race logic
- [x] Main menu
- [ ] Car selection UI
- [ ] Track selection UI
- [ ] Custom cars
