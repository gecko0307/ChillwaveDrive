# Chillwave Drive
Work-in-progress simcade racing game. It tries to stay both realistic and easy to control via keyboard or gamepad.

## About the Project

Chillwave Drive is written in [D language](https://dlang.org) using [Dagon Engine](https://github.com/gecko0307/dagon) and [Newton Game Dynamics](https://github.com/madeapps/newton-dynamics).

The game implements fairly realistic vehicle physics using Pacejka '94 tyre friction model. The model determines the material of the track under the car's wheels and adjusts itself accordingly. AI-controlled cars are supported; very accurate and stable path following algorithm is implemented, based on the Pure Pursuit method, as well as basic recovery mode. Opponents are capable of drifting, and the AI ​​can be configured to simulate different driving styles, from "proper" urban driving to risky and aggressive.

[![Screenshot1](https://blog.pixperfect.online/wp-content/uploads/2026/06/chillwave-drive-ai-2.jpg)](https://blog.pixperfect.online/wp-content/uploads/2026/06/chillwave-drive-ai-2.jpg)

Currently I'm working on the race logics, weather effects, and car description format.

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

## Credits

- [McLaren GT by carmodifier](https://sketchfab.com/3d-models/2020-mclaren-gt-9e436ba9d4924b719552f1889a5a6d87), available under CC BY-NC
- Music: ["Stellar Escape" by Prigida](https://uppbeat.io/track/prigida/stellar-escape) (from Uppbeat).
