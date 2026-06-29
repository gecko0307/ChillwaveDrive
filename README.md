# Chillwave Drive
Work-in-progress simcade racing game. It tries to stay both realistic and easy to control via keyboard or gamepad.

## About the Project

Chillwave Drive is written in [D language](https://dlang.org) using [Dagon Engine](https://github.com/gecko0307/dagon) and [Newton Game Dynamics](https://github.com/madeapps/newton-dynamics).

The game implements fairly realistic vehicle physics using Pacejka '94 tyre friction model. The model determines the material of the track under the car's wheels and adjusts itself accordingly. AI-controlled cars are supported; very accurate and stable path following algorithm is implemented, based on the Pure Pursuit method, as well as basic recovery mode. Opponents are capable of drifting, and the AI ​​can be configured to simulate different driving styles, from "proper" urban driving to risky and aggressive.

[![Screenshot1](https://blog.pixperfect.online/wp-content/uploads/2026/06/chillwave-drive-ai-2.jpg)](https://blog.pixperfect.online/wp-content/uploads/2026/06/chillwave-drive-ai-2.jpg)

Currently I'm working on the race logics, weather effects, and car description format.

## Controls

- Mouse - rotate the camera
- RMB - take a screenshot
- W / Right trigger - accelerate
- S / Left trigger - brake/reverse
- A / Left stick -X - steer left
- D / Left stick +X - steer right
- L - toggle headlights
- M - play/stop background music
- Escape - car editing mode
- F5 - show debug info.

Xbox controller is recommended. Steering wheels are not supported yet.

## TODO List

- [x] Vehicle physics
- [x] Sounds
- [x] Opponents AI
- [x] Road/ground material detection
- [ ] Race logic
- [ ] Game UI, main menu
- [ ] Car selection
- [ ] Track selection
- [ ] Tournament logic

## Credits

- [McLaren GT by carmodifier](https://sketchfab.com/3d-models/2020-mclaren-gt-9e436ba9d4924b719552f1889a5a6d87), available under CC BY-NC
- Music: ["Stellar Escape" by Prigida](https://uppbeat.io/track/prigida/stellar-escape) (from Uppbeat).
