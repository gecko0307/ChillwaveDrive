# Chillwave Drive
Work-in-progress vehicle simulator based on [Dagon Engine](https://github.com/gecko0307/dagon) and [Newton Game Dynamics](https://github.com/madeapps/newton-dynamics). It tries to stay both realistic and easy to control via keyboard or gamepad.

## About the Project

Chillwave Drive is written in [D language](https://dlang.org). The game implements its own wheel dynamics and friction model. When driving, dynamic friction (longitudinal and lateral) is applied based on the Pacejka '98 formulas, while when stopped, static friction is applied to prevent the car from sliding sideways on a slope.

AI-controlled cars are supported. Fairly accurate and stable path following algorithm is implemented, based on the Pure Pursuit method, as well as basic recovery mode. Opponents are capable of drifting, and the AI ​​can be configured to simulate different driving styles, from "proper" urban driving to risky and aggressive.

[![Screenshot1](https://blog.pixperfect.online/wp-content/uploads/2026/06/chillwave-drive-ai-1.jpg)](https://blog.pixperfect.online/wp-content/uploads/2026/06/chillwave-drive-ai-1.jpg)

## Controls

- Mouse - rotate the camera
- LMB - lock/unlock camera control
- RMB - take a screenshot
- W / Right trigger - accelerate
- S / Left trigger - brake/reverse
- A / Left stick -X - steer left
- D / Left stick +X - steer right
- L - toggle headlights
- M - play/stop background music
- Escape - exit.

Xbox controller is recommended.

## Credits

- [McLaren GT by carmodifier](https://sketchfab.com/3d-models/2020-mclaren-gt-9e436ba9d4924b719552f1889a5a6d87), available under CC BY-NC
- [Golden Gate Hills HDRI by Dimitrios Savva and Jarod Guest](https://polyhaven.com/a/golden_gate_hills), available under CC0
- Music: ["Stellar Escape" by Prigida](https://uppbeat.io/track/prigida/stellar-escape) (from Uppbeat).
