# Switch Robot

Navigate between robots in the simulator.

## Via JavaScript (preferred)

```javascript
window.location.href = '/<robot-name>'
```

Examples:
```javascript
window.location.href = '/UR5'
window.location.href = '/Meca500-R3'
window.location.href = '/ABB-IRB2400'
window.location.href = '/FANUC-LRMate200iD'
window.location.href = '/Kinova-Gen3'
```

This triggers a full page navigation. Wait ~2 seconds for the page to load and the 3D scene to initialize before interacting.

## Notes

- The URL path is the robot directory name (case-sensitive)
- After navigation, all element refs are invalidated — use `find` again for any UI interaction
- Camera and joint state reset to defaults on page load
- The robot dropdown at top-left also works but JS navigation is more reliable and faster
