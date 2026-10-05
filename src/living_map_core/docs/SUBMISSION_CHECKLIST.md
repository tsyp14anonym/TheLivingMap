# Phase 1 submission checklist (deadline 05/10/2026)

Source: the TSYP14 poster and the team sprint plan. **Aim to submit on 4 Oct, not on the last day**, and ask the organisers what time the deadline closes and where to upload (the poster only says "Submit your project").

## Deliverables
| Deliverable | Poster points | Status | What is left (owner) |
|---|---|---|---|
| GitHub repository | 4 | code, README, diagrams, docs ready in the project folder | create the repository, push, check a clean clone builds (Systems lead) |
| Simulation demo video | 8 | script ready for the one-floor building (`docs/VIDEO_SCRIPT.md`) | **record it**, 3 minutes at most (Simulation lead) |
| Short technical report, max 6 pages | 2 | **being regenerated** for the one-floor project from the final measurements (LaTeX sources in progress; the earlier PDF was removed because it contained two-floor numbers) | add team name and members; compile; reread (Team lead) |
| Detailed technical document (no page limit) | - | being written together with the report, same source of numbers | same |
| Implementation plan | 3 | `docs/PHASE2_IMPLEMENTATION_PLAN.md` + report section X | confirm BOM choices and add local prices (Team lead) |
| Failure cases | 3 | `docs/FAILURE_CASES.md` with measured results | reread; the sprint plan wants `FAILURE_CASES.md` in the repo (done) |
| Technical solution and architecture | in the 35 below | diagrams in `docs/diagrams/` | none |

Technical aspects (5 points each): Writer autonomy, event detection and beacon deposition, beacon message and signal design, frame translation, Outside Network Area, Executor robot, technical diagrams. All are covered in the report (sections III to VII, figures 1 to 3).

## Bonus (5 points) and administration
* Registration completed before the deadline (confirm).
* IEEE membership: 2 AESS members, 2 RAS members, 1 young professional with both memberships. Collect the 5 membership IDs (the form asks for them).

## Before you submit
1. Fill the placeholders on page 1 of the report: `[Team name]`, `[Member 1..4]`.
2. Build from a clean clone: `colcon build --symlink-install`, then `bash scripts/run_clean.sh speed:=6`. Check `bash scripts/check_run.sh` and `grep -c "CALLBACK CRASHED" ~/launch.log` is 0.
3. Run the tests: `python3 -m pytest test/test_core.py -q` (about 2 minutes).
4. Make sure the video does not claim Gazebo unless the Gazebo window really shows the building in your take.
5. Read the report once: every number can be reproduced with `python3 tools/collect_results.py`.

## Things to know about the documents (found while checking them)
* The poster says Phase 1 is worth 45 points, but its lines add up to more, and your sprint plan's table adds up differently. Ask the organisers how it is scored; deliver everything.
* The sprint plan's schedule ends on Oct 6, one day after the deadline.
* The sprint plan names `slam_toolbox` and Nav2; the code uses its own exploration and localisation model. The report says so (section XI).
* The sprint plan's formula P(t) has the score *falling* with age; the system keeps two clocks: observation confidence falls, and the priority number falls (more urgent) while an event waits. The report explains both (section IV and VI).

## Questions for the organisers (aess@ieee.tn, ras@ieee.tn)
1. Where and in which format do we upload (repository link, video, PDF)? What time does 05/10 close?
2. How are the 45 points of Phase 1 distributed?
3. Is the video limited in length or size?
