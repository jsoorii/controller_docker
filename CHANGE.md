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
