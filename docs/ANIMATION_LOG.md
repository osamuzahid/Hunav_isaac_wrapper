# Agent animation & locomotion log (isaac-social-nav)

Sections are in **discovery order** (how we found each failure on the Isaac 6.0.1 + Jazzy port, then how we fixed it). Code markers: search `ORIGINALLY` / `PATCH (isaac-social-nav)` in `hunav_manager.py` and `animation_utils.py`.

Parent handoff / validation IDs: main repo `docs/ENVIRONMENT.md` (Validated **#26–#28**, port **6b–6d**).

---

## 0. Baseline (upstream v2.0)

Upstream authored an Idle/Walk **AnimationGraph**, retargeted clips from `Biped_Setup` onto People skins, and drove a float variable `speed` from HuNav linear velocity. That worked on Isaac **4.5 / Humble**. On **6.0.1** the same path failed in several stacked ways (below).

---

## 1. AnimGraph `speed` type mismatch (early port)

| | |
|---|---|
| **Symptom** | Log spam: `Failed to compile` / `Type mismatch` on AnimGraph `speed`; agents not blending walk. |
| **Cause** | Isaac 6 AnimGraph expects the graph variable as a **uniform float**. Upstream authored a non-uniform / wrong variability token. |
| **Fix** | In `create_agent_animation_graph` (`animation_utils.py`): declare custom **uniform** float `speed`; mark `ReadVariable` `inputs:variableName` custom + `VariabilityUniform`. Cast to `float` in `set_anim_graph_speed`. |
| **Files** | `animation_utils.py` |
| **Status** | Done (port polish). Compile / type-mismatch count **0** on museum smoke. |

---

## 2. Extensions not loaded / Kit crash on enable

| | |
|---|---|
| **Symptom** | `ModuleNotFoundError: omni.anim.graph.core`, or Kit **exit 139** if extensions enabled mid-run with `app.update()`. |
| **Cause** | `isaacsim.exp.base.python` does not load anim graph/retarget by default. Pumping the app after runtime `enable_extension` crashed OmniGraph on this host. |
| **Fix** | Enable `omni.anim.graph.core` + `omni.anim.retarget.core` at **Kit start** via `sim_app_config` `extra_args`; also request them before `import omni.anim.graph.core as ag` — **do not** call `app.update()` after enable. |
| **Files** | `sim_app_config.py`, `hunav_manager.py` (top) |
| **Status** | Done. |

---

## 3. T-pose — wrong retarget skeleton (Validated #26 / port 6b)

| | |
|---|---|
| **Symptom** | Characters upright but frozen in **bind / T-pose**. `get_character` OK; `speed` updates; joints static. |
| **Diagnosis** | Compared retarget tag overlap: AnimGraph-test `Biped_Setup` shares only **1** tag (`Head`) with Isaac 6 RL People skins (`RL_BoneRoot`, ~101 joints). Retargeted clips were near-static (museum pose matched idle bind). |
| **False lead** | First CDN remap used `Assets/AnimGraph/105.0/Test/Graph/Isaac/Biped_Setup.usd` because Isaac 6 removed `Isaac/People/Characters/Biped_Setup.usd` from the local tree (HTTP 404). That path is **alive** but **wrong skeleton**. |
| **Fix** | Point `default_biped_usd` at Isaac **5.1** People biped (still HTTP 200): `…/Assets/Isaac/5.1/Isaac/People/Characters/Biped_Setup.usd` (~**51** shared tags with RL skins). |
| **Files** | `hunav_manager.py` (`default_biped_usd`) |
| **Verify** | Headless probe: both `F_Business_02` and `M_Medical_01` `joints_changed=True` once clips also file-referenced (step 4). |

---

## 4. T-pose — inline SkelAnimation not played by AnimGraph (Validated #26 / port 6b)

| | |
|---|---|
| **Symptom** | Even with a good biped + motion in USD time samples, AnimationClip still left characters in bind pose. |
| **Diagnosis** | NVIDIA `play_animation.usda` works (file-referenced clips). Runtime `CreateRetargetAnimationsCommand` writes **inline** `SkelAnimation` prims — Isaac **6.0** AnimGraph does not advance those the same way. |
| **Fix** | After retarget, `materialize_retargeted_animation_references()` exports Idle/Walk to `/tmp/hunav_isaac_retarget/<agent>/…skelanim.usd` and re-`AddReference`s them in place. Per-agent retarget (don’t share one skin’s clips across dissimilar RL skeletons). |
| **Files** | `animation_utils.py` (`setup_anim_retargeting`, `materialize_retargeted_animation_references`) |
| **Verify** | Museum batch: retarget USDs written; AnimGraph compile/speed mismatch **0**; joints move under Idle/Walk + `speed`. |

---

## 5. Standing idle — no SFM goals (Validated #27 / port 6c)

| | |
|---|---|
| **Symptom** | After T-pose fix: **not** T-posing, but both agents **just standing**. Live `/people`: `|v_xy| ≈ 0`, `behavior` tag **`0`**. |
| **Cause** | Upstream `_create_agent_msg` never filled `Agent.goals` from YAML `global_goals`. `AgentManager.initializeAgents` only copies goals on first `/compute_agents` call — empty goals ⇒ no attractive force ⇒ Idle blend. Also `behavior.type` was commented out (defaults to 0). |
| **Secondary** | Agent rigid bodies had gravity → sinking through floor (z drift), which made motion look even worse. |
| **Fix** | Resolve `global_goals` + agent goal ids into `geometry_msgs/Pose[]`; parse `behavior.type` (int or name like `Regular`); kinematic + `disableGravity`; pin Z to spawn height. Quote goal ids as **strings** in YAML (`hunav_loader` expects `string_array`). |
| **Files** | `hunav_manager.py` (`_resolve_agent_goals`, `_create_agent_msg`, spawn physics); `museum_agents.yaml` / `empty_world_agents.yaml` |
| **Verify** | Headless museum: behavior=`1`, Agent2 `|v_xy| ~ 0.9`. |

---

## 6. Wall jam + frantic spin (Validated #28 / port 6d)

| | |
|---|---|
| **Symptom** | Female stood near a wall; male walked then **spun in place** on geometry. `/people`: poses stuck, `|v_xy|≈0`, wild yaw. |
| **Cause (goals)** | Early museum goals (e.g. `(-8,-6)`, `(6,0)`) sat in **low-clearance** map cells; agents walked into walls and SFM could not advance. |
| **Cause (rays)** | People character meshes ship with colliders. Obstacle **raycasts** hit the agent (and peers) → fake “walls” all around → goal vs obstacle thrash → spin. |
| **Fix (first pass)** | (1) Retarget goals to free-space. (2) `_disable_collisions_recursive`. (3) Ignore `/World/Characters` ray hits. (4) Freeze yaw when nearly stopped. |
| **Files** | `hunav_manager.py`, `src/scenarios/museum_agents.yaml` |

### 6b. Follow-up — still jam after first pass

| | |
|---|---|
| **Symptom** | After relaunch: both run into a wall; female stops; male still rotates L/R. Live: Agent1 on **occupied map cell** `(2.75, 3.47)` val=0; Agent2 `|v_xy|~0.3` next to a wall (yaw freeze at 0.05 too low); `/people` showed huge `velocity.z` (PhysX tumble fed back into msgs). |
| **Root cause** | HuNav SFM is a **local** force model — it walks **straight lines** between goals. Goals in different rooms ⇒ path crosses walls. Obstacle rays against the museum mesh then pin/spin agents. Map “free” clearance ≠ clear line-of-sight. |
| **Fix** | (1) All museum spawns/goals inside one **mutual-LOS** room (`x∈[-8,4], y∈[-9.5,2.5]`). (2) `ignore_obstacle_rays: true` in `museum_agents.yaml` (no navmesh yet). (3) Zero planar-only velocities (kill `v_z` / tumble). (4) Freeze yaw when `|v_xy| &lt; 0.2`. |
| **Trade-off** | Without obstacle rays, agents can clip through mesh if a goal is wrong; keep goals LOS-checked. Real wall avoidance waits on Nav2 / a planner. |
| **Verify** | **DONE (GUI)** — both agents walk cyclic goal loops in the west open room (expected HuNav Regular + `cyclic_goals: true`). |

---

## Quick reference — files that matter

| File | Role |
|---|---|
| `animation_utils.py` | Graph authoring, `speed` uniform float, retarget + materialize clips, `set_anim_graph_speed` |
| `hunav_manager.py` | Biped USD, spawn/physics, goals → `/compute_agents`, obstacle rays, orientation / Z pin |
| `sim_app_config.py` | Kit-start `--enable` for anim extensions + laptop profiles |
| `src/scenarios/museum_agents.yaml` | Spawns, goal loop, behavior type ints, string goal ids |
| `src/worlds/assets/museum/PATCH_NOTES.md` | Museum **look** patches (floor/light), not animation |

---

## Operator checks

```bash
# After agents are live (ROS_DOMAIN_ID matching your launch):
ros2 topic echo /people --once
# Expect: behavior tag '1', non-trivial position.x/y change over a few seconds,
# |velocity.x/y| rising when walking (Idle when briefly at a goal is OK).
```

Logs to skim on launch:

- `Behavior trees succesfully initiated!` / `Agents received: 2`
- `Created Animation` / retarget paths under `/tmp/hunav_isaac_retarget/`
- **Absence** of repeated `Failed to compile` / `Type mismatch` on `speed`
- No flood of PhysX invalid inertia on agents after collider disable

---

## Do not reintroduce

- AnimGraph-test `Biped_Setup` as retarget source for RL People skins  
- Inline-only retargeted `SkelAnimation` (must materialize to file refs on Isaac 6.0)  
- Runtime `enable_extension` + `app.update()` for anim  
- Empty `Agent.goals` on first `/compute_agents`  
- Leaving character mesh colliders enabled for HuNav raycast obstacles  
