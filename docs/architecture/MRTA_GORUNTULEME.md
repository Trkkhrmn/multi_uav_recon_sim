# Viewing MRTA Visualization

MRTA (target assignments, drone states, assignment lines) can be followed in two ways:

---

## 1. Seeing the markers in RViz

### With the provided config (easiest)

1. Run the sim and MRTA (Gazebo + PX4 + scout + MRTA node).
2. In a new terminal:
   ```bash
   cd ~/multi_uav_recon_ws && source install/setup.bash
   rviz2 -d src/task_allocation/config/mrta.rviz
   ```
3. **Fixed Frame** should be `world`, **MRTA** display is tied to `/mrta/markers`.
4. On the map you'll see:
   - **Green** box: Targets not visited yet (at drop altitude).
   - **Grey** box: Targets where a payload was dropped (visited).
   - **Yellow lines**: Drone current position -> assigned target.
   - **White labels**: State per drone (e.g. `W5: GO`, `W6: DESCEND`).

### Opening RViz and adding the topic by hand

1. Run `rviz2`.
2. **Global Options** -> **Fixed Frame** -> set to `world`.
3. **Add** (bottom left) -> **By topic** -> `/mrta/markers` -> **MarkerArray** -> **OK**.
4. Adjust view with **Views** if needed (Orbit to zoom/rotate).

---

## 2. Text summary (/mrta/status)

Live summary in terminal:

```bash
ros2 topic echo /mrta/status
```

Example:

```
data: 'W5->(189,56) GO | W6 IDLE | W7->(191,58) DESCEND | W8 RTH_LANDED'
```

So you see which drone is going to which target and in which state.

---

## 3. Markers not showing

- Is MRTA node running: `ros2 node list | grep mrta`
- Is the topic there: `ros2 topic list | grep mrta`
- Marker publish: `ros2 topic echo /mrta/markers --once`

Fixed Frame must be `world` (or whatever main frame you use); if it's wrong, markers won't show.
