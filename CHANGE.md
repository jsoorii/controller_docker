# Change Log

| Date | Description | Rollback Commit |
|------|-------------|-----------------|
| 2026-04-07 | Added ROS2 pub/sub interface to main_test.py for real-time UR5e command sending. | `92001ff` |
| 2026-04-08 | Moved AUTH_USERNAME/AUTH_PASSWORD from hardcoded values to .env file in web_dashboard. | `3a494c1` |
| 2026-04-08 | Fixed JS auth for API calls: added apiFetch() wrapper and replaced EventSource with fetch+ReadableStream for log streaming. | `3a494c1` |
| 2026-04-09 | Fixed cloudflared zombie process accumulation and added auto-start dashboard tunnel (port 8765) in app lifespan alongside meshcat tunnel. | `3a494c1` |
| 2026-04-09 | Fixed render_snapshot.py camera: added MjvCamera with lookat/distance/azimuth/elevation to prevent robot being clipped. | `3a494c1` |
| 2026-04-09 | Separated dashboard tunnel from app lifecycle: moved cloudflared(8765) to heroi-tunnel.service, dashboard to heroi-dashboard.service; removed reload=True. | `3a494c1` |
| 2026-04-09 | Added 3D turntable robot viewer: render_snapshot.py --frames mode (24 JPEG frames), /api/robot_3d endpoint, drag-to-rotate UI in dashboard. | `3a494c1` |
| 2026-04-09 | Extended 3D viewer to 2-axis rotation: azimuth×elevation grid (24×5=120 frames), horizontal drag=azimuth, vertical drag=elevation. | `3a494c1` |
| 2026-04-08 | Updated web_dashboard README with full API docs, code structure, and change history. | `3a494c1` |
| 2026-04-09 | Reduced 3D viewer frames from 120 to 60 by halving N_AZ (24→12, 30° steps); adjusted drag sensitivity /12→/24 to maintain same angular speed. | `5cba1fc` |
| 2026-04-13 | Fixed double login: removed HTTP Basic auth from GET / so browser native dialog no longer appears; custom HTML overlay is now the sole login UI; API endpoints retain server-side auth. | `5cba1fc` |
| 2026-04-13 | Added background run commands (nohup, log, pgrep, pkill) to web_dashboard README. | `5cba1fc` |
| 2026-04-13 | Added launch.sh: single script to start dashboard server + Cloudflare tunnel with status output. | `5cba1fc` |
| 2026-04-13 | Rewrote web_dashboard README: added launch.sh usage, address check commands, full API table, reorganized sections. | `5cba1fc` |
| 2026-04-13 | Rewrote root README.md: reflects full current state including web dashboard, 3D viewer, Cloudflare tunnel, and API table. | `5cba1fc` |
| 2026-04-13 | Updated README: added ROS2 interface table (joint/ee_target/mode sub, joint/ee_pose pub), task-space IK; corrected Next Milestones to exclude already-implemented features. | `5cba1fc` |
| 2026-04-13 | Updated launch.sh: added `stop` subcommand to kill all processes at once; added Meshcat local/tunnel URL display on startup; fixed Meshcat tunnel pkill scope. | `5cba1fc` |
| 2026-04-13 | Improved launch.sh Meshcat external URL display: separated into step 4/4 with 20s timeout, using sed instead of double-grep for reliability. | `5cba1fc` |
| 2026-04-13 | Moved Meshcat tunnel management from main.py to launch.sh; removed _start_meshcat_tunnel, lifespan, and /api/meshcat_url from main.py; launch.sh now directly starts cloudflared for port 7000 with dedicated log /tmp/heroi-meshcat-tunnel.log. | `5cba1fc` |
| 2026-04-13 | Removed separate Meshcat Cloudflare tunnel; Meshcat now accessed via dashboard proxy at /meshcat (wss:// rewriting already handled); launch.sh [4/4] now prints dashboard_url/meshcat directly. | `5cba1fc` |
| 2026-04-13 | Fixed Meshcat white screen: rewrote src="main.min.js" to absolute path /meshcat/main.min.js in meshcat_index() so browser loads JS through meshcat_static() handler which rewrites ws:// to wss://. | `5cba1fc` |
| 2026-04-13 | Fixed launch.sh pkill pattern: changed "python3 main.py" to "web_dashboard/main.py" to match actual process command line when launched via absolute path. | `5cba1fc` |
| 2026-04-13 | Clarified launch.sh stop message: simulation (main_test.py) runs inside Docker container and must be stopped separately via dashboard exec. | `5cba1fc` |
| 2026-04-13 | Fixed joint command reliability: changed `ros2 topic pub --once` to `--times 3` in /api/pub/joint to prevent message loss before subscriber discovery. | `5cba1fc` |
