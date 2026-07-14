I am working on a Unity/Python digital twin project for a SCORBOT/ER9Pro-style robot arm with human–robot safety monitoring. The system already uses Unity for robot visualization, MQTT/UDP communication, YOLO pose estimation, dual-camera human tracking, and safety-distance logic.

I also have a Wolfram Mathematica notebook (Kinematics.md) that implements a simplified 3-link planar robot arm model with forward kinematics, inverse kinematics, Jacobian calculation, Gaussian elimination, and Newton–Raphson IK. I want to integrate and expand this into the current digital twin project as a proper kinematic modeling layer.

Your task:

1. Inspect the existing repository structure first.
2. Find the Unity scripts related to robot joint control, robot visualization, safety logic, MQTT/UDP receivers, and human skeleton visualization.
3. Add a complete robot kinematics module that supports:

   * configurable robot link lengths and joint offsets;
   * forward kinematics from joint angles to robot joint/link positions;
   * end-effector pose calculation;
   * robot link segment generation for distance checking;
   * optional inverse kinematics based on the existing Mathematica logic;
   * clear separation between model parameters, math functions, and Unity visualization.
4. Do not hard-code uncertain ER9Pro dimensions as final truth. If exact DH parameters or link lengths are not available in the repo, create a configurable JSON/ScriptableObject/config file with clearly labeled placeholder values and comments explaining where real measured values should be inserted.
5. Implement the main model preferably in Unity C# so the digital twin can directly compute robot geometry from live joint telemetry.
6. If useful, also provide a Python version of the same forward-kinematics logic so the safety pipeline can compute robot geometry before sending UDP safety packets to Unity.
7. Replace or extend the current safety-distance calculation so it can compute minimum distance between:

   * human 3D skeleton joints/keypoints;
   * robot link segments generated from forward kinematics.
8. Implement the distance calculation using point-to-line-segment distance:
   d_min = min distance between each human joint and each robot link segment.
9. Preserve the existing safety logic:

   * warning state;
   * stop threshold;
   * release threshold;
   * hysteresis;
   * stale-data unsafe behavior;
   * gesture stop/go logic, if already implemented.
10. Add validation/debug tools:

* show robot joint positions as debug spheres in Unity;
* show robot link segments as debug lines;
* display current end-effector position;
* display minimum human–robot distance;
* display which robot link and human joint produced the minimum distance.

11. Add a small validation scene or script that tests several joint-angle configurations and prints:

* input joint angles;
* computed joint positions;
* end-effector position;
* minimum distance to a sample human point.

12. Add unit tests or simple test scripts for:

* forward kinematics consistency;
* point-to-segment distance;
* safety threshold decision;
* invalid/stale data handling.

13. Keep existing functionality working. Do not delete existing MQTT/UDP/Unity logic unless necessary. Prefer adding new scripts and modifying existing ones minimally.
14. Add comments explaining the math clearly enough for an academic paper.
15. At the end, summarize:

* files changed;
* new files added;
* how to configure robot dimensions;
* how to run the validation test;
* how the new kinematic model connects to the safety pipeline.

Important framing:
This is for a journal article. The goal is not only to make the robot move visually, but to make the digital twin more scientifically defensible by adding a model-based robot geometry layer. The final system should support this claim:

“The robot is represented as a set of forward-kinematic link segments derived from joint telemetry, enabling human–robot distance estimation between reconstructed human skeleton joints and the robot body geometry, rather than relying only on coarse robot bounding boxes or detected robot centers.”

Dont edit original file
