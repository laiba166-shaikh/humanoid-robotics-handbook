# Code Example Patterns

Every code block in a lesson MUST follow the four-part pattern for clarity and consistency.

## The Four-Part Pattern

```markdown
**What we're building**: [1 sentence — the problem this code solves]

\`\`\`python
# [filename hint: e.g., velocity_publisher.py]
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
\`\`\`

**Expected output**:
\`\`\`
[INFO] Created publisher on topic /cmd_vel with queue size 10
\`\`\`

**What's happening**:
- Line 6: We inherit from `Node`, the base class for all ROS 2 executables
- Line 8: `create_publisher(msg_type, topic, queue_size)` registers on the topic
```

## Code Rules

| Rule | Rationale |
|------|-----------|
| Max 40 lines per block | Longer blocks lose attention |
| Show all imports | Never assume the reader knows them |
| Comments explain "why", not "what" | Code shows what; comments explain reasoning |
| Meaningful variable names | `velocity_publisher` not `vp` or `pub1` |
| Filename hint in first comment | Reader knows where to save the file |
| Expected output always shown | Reader can verify their code works |

## Safety Code Pattern

For lessons involving motor control, hardware, or physical systems, add safety blocks:

```markdown
:::danger Safety Warning
Always set joint velocity limits before running on physical hardware.
Never skip the simulation step.
:::

**What we're building**: A velocity-limited motor controller

\`\`\`python
# safe_motor_controller.py
MAX_VELOCITY = 0.5  # rad/s — safety limit

def send_command(velocity: float) -> float:
    """Clamp velocity to safe range before sending."""
    safe_vel = max(-MAX_VELOCITY, min(MAX_VELOCITY, velocity))
    if abs(velocity) > MAX_VELOCITY:
        print(f"WARNING: Clamped {velocity} to {safe_vel}")
    return safe_vel
\`\`\`
```

## Progressive Code Pattern

When building on prior code, show only the new additions with clear markers:

```markdown
**What we're adding**: Error handling to our publisher

\`\`\`python
# velocity_publisher.py (updated)
# ... previous code unchanged ...

    def publish_velocity(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: linear={linear_x}, angular={angular_z}')
\`\`\`
```

## Language Support

While Python is the default language, the four-part pattern applies to all languages:
- Always show imports/includes
- Always show expected output
- Always explain key lines
- Always include a filename hint
