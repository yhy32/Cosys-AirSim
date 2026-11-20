# Go2 local collision in Blocks

Blocks checks the Go2 torso and head against query-enabled `WorldStatic` geometry that blocks `Pawn`. Ordinary walls need no tag. On startup the bridge reads each dog's current pose and joint angles from UE with `getGo2State`. Running the script again in the same Play session starts the selected maneuver from that pose, with a fresh gait cycle and zero initial velocity. Starting a new Play session spawns the dogs at their configured locations. During walking, each `moveGo2State` call sweeps the body box, allows motion along the wall, and returns any correction in the same spawn frame. The bridge applies the accepted pose and removes inward velocity before continuing the simulation.

1. For foot or leg contact with a raised step, add `Go2Obstacle` under **Details → Actor → Tags**. You can also tag just the collision component. The component needs query collision enabled and the `WorldStatic` object type. This additionally places a collision box in MuJoCo so the step can support the dog.
2. For a simple, axis-aligned wall or step, a collidable mesh works as-is. For a rotated wall or irregular mesh, add and tag a **Box Collision** component sized to the intended obstacle. The bridge uses the box's rotation and dimensions. Other components use their world bounds.
3. Leave `Z` unset to align the Go2 spawn with static ground below it. If you specify `Z`, place the spawn origin on the walking surface. Leave the floor untagged because MuJoCo already has a plane at the spawn origin.
4. Run the bridge from `mujoco_env` while Blocks is playing in Go2 or Hybrid mode.

```powershell
python PythonClient/go2_bridge.py --scene C:/BUPT/mujoco_menagerie/unitree_go2/scene.xml
```

The Go2 example settings place the two dogs 2 m apart on open ground near Player Start in the default `FlyingExampleMap`. `go2` walks forward, and `go2_2` turns before walking. The bridge drives both by default. Use `--vehicle go2_2` to drive only the second dog, or `--maneuver walk` to override the selected gait. Each dog has its own MuJoCo simulation and local obstacle query.

The Hybrid example in `settings_hybrid_example.json` adds three cars, one CPHusky and two drones to the same scene. Run `PythonClient/hybrid_demo.py` to move all eight vehicles together. To walk just the dogs, run `PythonClient/go2_bridge.py`. It discovers the active Go2s by type through `listGo2Vehicles`.

Run these commands from the repository root. The bridge uses the Unitree Go2 scene and its meshes from a separate MuJoCo Menagerie checkout. If Menagerie lives elsewhere, pass that checkout's `unitree_go2/scene.xml` using `--scene`. The combined Hybrid demo uses the bridge's default scene path.

The default tagged-box entry radius is 0.65 m, the exit radius is 25% larger, and the bridge queries up to 10 times per simulated second. Use `--obstacle-radius 0` to test UE body sweeps alone. The bridge reuses collision boxes and recompiles with state preserved only when it needs a box of a different size.

Go2 uses a torso box and capsule colliders on the head, hips, thighs, and calves for contact with synchronized UE obstacles. The native MuJoCo ground and foot contacts still handle walking on the floor. Synchronized obstacle dimensions are used by default. Use `--obstacle-clearance 0.05` for optional horizontal padding, or `--debug-obstacles` to see which boxes are in range and making contact.

With `--debug-obstacles`, `UE body hit` reports a body correction from the map, and `MuJoCo obstacle contact` reports contact with a tagged box. The ground plane and the leg joints are simulated in MuJoCo. This setup targets static scenery.
