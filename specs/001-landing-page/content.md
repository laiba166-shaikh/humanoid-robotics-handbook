# Content Specification: Textbook Landing Page

**Source**: Approved from `landing-page.md` (root)
**Date**: 2026-02-06
**Status**: Approved

This file is the single source of truth for all landing page copy. Implementation must match this content exactly (SC-008).

---

## Section 1: Hero

- **Eyebrow**: `PHYSICAL AI & HUMANOID ROBOTICS` (uppercase, letter-spacing 2px, 0.875rem, Pacific Blue)
- **Heading (H1)**: `From ChatGPT to Walking Robots` (3.5rem desktop, 2.25rem mobile, white)
- **Subtitle**: "Learn to build AI systems that move through the physical world. This 13-week course takes you from ROS 2 fundamentals through simulation and perception to deploying humanoid robots with voice-controlled intelligence." (1.25rem, Azure Mist at 85% opacity)
- **Primary CTA**: `Start Learning` → `/docs/module-1-ros2` (button--primary button--lg)
- **Secondary CTA**: `Explore Modules` → scrolls to `#modules` (button button--secondary button--lg)

---

## Section 2: Stats Bar

| Number | Label |
|--------|-------|
| **13** | Weeks |
| **4** | Modules |
| **40** | Lessons |
| **4** | Hardware Tiers |

- Numbers: 2.5rem, bold, `var(--ifm-color-primary)`
- Labels: 0.875rem, uppercase, letter-spacing 1px

---

## Section 3: Why Physical AI? (3 Value Cards)

**Section heading**: `Why Physical AI?`
**Subheading**: `The next frontier of artificial intelligence extends beyond screens into the physical world.`

### Card 1 — The $150B Opportunity

> Goldman Sachs projects the humanoid robot market will reach $154 billion by 2035. Companies including Tesla, Figure, NVIDIA, and Boston Dynamics are racing to deploy humanoid workers. Engineers who understand Physical AI will shape this industry.

### Card 2 — Where Software Meets Hardware

> Large Language Models can reason and plan. Physical AI gives them a body. This course teaches the complete stack: from ROS 2 middleware and physics simulation to perception pipelines and voice-controlled robot actions.

### Card 3 — Learn by Building

> Every module produces working artifacts. You will build ROS 2 packages, simulate robots in Gazebo, train perception models with NVIDIA Isaac, and deploy a voice-controlled humanoid in your capstone project.

---

## Section 4: Module Preview Cards

**Anchor**: `id="modules"`

| Module | Title | Focus | Weeks | Tiers | Status |
|--------|-------|-------|-------|-------|--------|
| 1 | The Robotic Nervous System | ROS 2 Middleware | Weeks 1-5 | 1-2 | **Available** (green pill) |
| 2 | The Digital Twin | Gazebo & Unity Simulation | Weeks 6-7 | 2 | Coming Soon (gray pill) |
| 3 | The AI-Robot Brain | NVIDIA Isaac Platform | Weeks 8-10 | 2-3 | Coming Soon |
| 4 | Vision-Language-Action | LLMs Meet Robotics | Weeks 11-13 | 3-4 | Coming Soon |

- Module 1: clickable, links to `/docs/module-1-ros2`
- Modules 2-4: opacity 0.7, not clickable
- Tier badges: reuse global `.tier-badge--{n}` classes

---

## Section 5: Choose Your Path (Hardware Tiers)

**Section heading**: `Choose Your Path`
**Subheading**: `This course works at every equipment level. Start where you are and scale up when ready.`

### Tier 1 — Cloud Explorer (highlighted, "Recommended Start" badge)

- **Equipment**: Any laptop with a browser
- **Cost**: $0
- **Description**: Follow all conceptual lessons, run ROS 2 through cloud environments like Foxglove Studio and The Construct, and complete browser-based exercises.

### Tier 2 — Local Developer

- **Equipment**: RTX GPU + Ubuntu 22.04
- **Cost**: $1,500-3,000
- **Description**: Run ROS 2 Humble natively, launch Gazebo simulations, train models locally, and build full development workstations.

### Tier 3 — Edge Deployer

- **Equipment**: Jetson Orin Nano + RealSense Camera
- **Cost**: ~$700
- **Description**: Deploy perception models to edge hardware, run real-time inference, and connect physical sensors for SLAM and navigation.

### Tier 4 — Robot Operator

- **Equipment**: Unitree Go2 or G1 Humanoid
- **Cost**: $3,000-16,000+
- **Description**: Execute sim-to-real transfer, control physical robots with ROS 2 nodes, and complete the capstone with a walking humanoid.

---

## Section 6: Learning Outcomes

**Section heading**: `What You Will Be Able to Do`
**Subheading**: `By the end of this course, you will have the skills to build intelligent physical systems.`

1. Explain Physical AI principles and the transition from digital to embodied intelligence
2. Build and deploy ROS 2 nodes, services, and actions for robotic control systems
3. Simulate humanoid robots in Gazebo and Unity with realistic physics and sensors
4. Develop perception pipelines using NVIDIA Isaac for visual SLAM and navigation
5. Design humanoid robot kinematics for bipedal locomotion and object manipulation
6. Integrate large language models with ROS 2 for voice-controlled robot actions

- Numbered circles: 36px round, Pacific Blue bg, Jet Black text, flexbox row with gap

---

## Section 7: FAQ

**Section heading**: `Frequently Asked Questions`

### Q1: Do I need a powerful GPU to take this course?

No. Every lesson includes a Tier 1 path that works with any laptop and a web browser. You can complete the conceptual content and many hands-on exercises using cloud-based tools like Foxglove Studio and The Construct. A local GPU becomes valuable in Module 2 for physics simulation, but it is not required to start learning.

### Q2: What programming experience do I need?

You need working knowledge of Python, including functions, classes, and basic data structures. Experience with Linux command-line tools (navigating directories, running scripts) will help. No prior robotics experience is required.

### Q3: Is this course only theoretical, or will I build real projects?

You will build working projects in every module. Module 1 produces ROS 2 packages. Module 2 creates robot simulations. Module 3 builds perception pipelines. The capstone in Module 4 produces a voice-controlled humanoid robot that navigates obstacles and manipulates objects.

### Q4: Which ROS 2 distribution does this course use?

This course targets ROS 2 Humble Hawksbill, the current Long-Term Support release running on Ubuntu 22.04. Code examples are also compatible with ROS 2 Iron where noted.

### Q5: Can I use macOS or Windows instead of Ubuntu?

ROS 2 runs natively on Ubuntu 22.04. For macOS and Windows users, the recommended approach is a dual-boot setup or a virtual machine. Docker-based workflows are possible but add complexity. The Tier 1 cloud path works on any operating system.

### Q6: How long does the full course take to complete?

The course spans 13 weeks with approximately 8-10 hours of study per week. Each lesson includes a duration estimate so you can plan your sessions. You can study at your own pace, but the weekly structure provides a recommended cadence.
