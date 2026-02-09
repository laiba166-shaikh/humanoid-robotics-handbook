# H2 Token Remediation Plan: Module 1 Lessons

**Date**: 2026-02-09
**Issue**: Only 35% of H2 sections (45/128) meet the 300-500 token RAG optimization requirement
**Impact**: Suboptimal RAG retrieval performance - sections too short lack context, sections too long exceed chunk size

---

## Executive Summary

All 12 Module 1 lessons require restructuring to meet the constitution's RAG optimization requirement of 300-500 tokens per H2 section. The issue is systemic across two categories:

1. **Structural sections** (Learning Objectives, Key Takeaways, etc.) - consistently too short
2. **Code-heavy instructional sections** - frequently too long

**Estimated Effort**: 12-15 hours (1-1.5 hours per lesson)

---

## Issue Breakdown

### Pattern 1: Structural Sections (Too Short)

**Affected sections in every lesson:**
- Learning Objectives: 79-97 tokens (target: 300-500)
- Why/Introduction: 79-134 tokens
- Key Takeaways: 144-241 tokens
- Check Your Understanding: 133-204 tokens
- Next Steps: 129-210 tokens

**Root cause**: These sections follow a minimalist template that prioritizes brevity over RAG optimization.

**Solution**: Expand each section with contextual information while maintaining pedagogical value.

### Pattern 2: Code-Heavy Sections (Too Long)

**Top offenders:**
- "ROS 2 Actions for Long-Running Tasks" - 950 tokens (lesson-4-services-actions.md)
- "Managing Agent State Across Callbacks" - 936 tokens (lesson-2-rclpy-agents.md)
- "Building a Humanoid Upper Body" - 928 tokens (lesson-4-urdf-humanoids.md)
- "Structuring an AI Agent Node" - 886 tokens (lesson-2-rclpy-agents.md)
- "Integrating Machine Learning Models" - 851 tokens (lesson-2-rclpy-agents.md)

**Root cause**: Code examples + explanatory text exceed optimal chunk size.

**Solution**: Split into multiple H2 sections or move code to separate subsections (H3).

---

## Remediation Strategy

### Phase 1: Template Revision (1-2 hours)

Create updated section templates that meet token requirements:

#### Learning Objectives Template (Target: 350 tokens)
```markdown
## Learning Objectives

[2-3 sentence introduction explaining what students will learn and why it matters]

By the end of this lesson, you will be able to:

- [Bloom's verb] [measurable outcome] [context/application]
- [Bloom's verb] [measurable outcome] [context/application]
- [Bloom's verb] [measurable outcome] [context/application]
- [Bloom's verb] [measurable outcome] [context/application]

[2-3 sentences connecting objectives to real-world applications or next lessons]

[1-2 sentences on prerequisite knowledge and how this builds on it]
```

#### Key Takeaways Template (Target: 350 tokens)
```markdown
## Key Takeaways

[2-3 sentence summary of the lesson's core message]

Here are the essential concepts to remember:

- **[Concept 1]**: [2-3 sentence explanation with context and application]
- **[Concept 2]**: [2-3 sentence explanation with context and application]
- **[Concept 3]**: [2-3 sentence explanation with context and application]
- **[Concept 4]**: [2-3 sentence explanation with context and application]
- **[Concept 5]**: [2-3 sentence explanation with context and application]

[2-3 sentences on how these concepts connect to broader robotics principles]
```

#### Check Your Understanding Template (Target: 350 tokens)
```markdown
## Check Your Understanding

[2-3 sentence introduction explaining the purpose of self-assessment]

Test your comprehension with these questions:

1. **[Question]**
   - [Brief hint or context for the question]

2. **[Question]**
   - [Brief hint or context for the question]

3. **[Question]**
   - [Brief hint or context for the question]

4. **[Question]**
   - [Brief hint or context for the question]

5. **[Question]**
   - [Brief hint or context for the question]

[2-3 sentences on how to use these questions for self-study and what to review if struggling]
```

### Phase 2: Lesson-by-Lesson Revision (10-12 hours)

**Priority Order:**

#### Tier 1: High-Impact Lessons (4 hours)
1. **lesson-1-foundations.md** - First lesson, sets expectations
2. **lesson-2-nodes.md** - Core ROS 2 concept
3. **lesson-3-topics-messages.md** - Most common pattern
4. **lesson-2-rclpy-agents.md** - Has 3 sections >850 tokens

#### Tier 2: Code-Heavy Lessons (4 hours)
5. **lesson-4-services-actions.md** - 3 sections >620 tokens
6. **lesson-4-urdf-humanoids.md** - 3 sections >750 tokens
7. **lesson-1-packages-python.md** - Package structure critical
8. **lesson-3-launch-params.md** - Launch files essential

#### Tier 3: Conceptual Lessons (3 hours)
9. **lesson-2-digital-to-physical.md** - 4 sections >500 tokens
10. **lesson-3-humanoid-landscape.md** - Market overview
11. **lesson-4-sensor-systems.md** - Sensor fundamentals
12. **lesson-1-core-concepts.md** - ROS 2 architecture

### Phase 3: Validation & Testing (1 hour)

- Re-run H2 token validation
- Verify all sections 300-500 tokens
- Test RAG retrieval with sample queries
- Confirm build still passes

---

## Specific Fixes by Section Type

### Structural Sections

#### Learning Objectives (Currently 79-97 tokens → Target 350)
**Add:**
- Introduction paragraph (50-75 tokens)
- Context for each objective (10-15 tokens per objective)
- Connection to real-world applications (50-75 tokens)
- Prerequisites reminder (30-50 tokens)

#### Key Takeaways (Currently 144-241 tokens → Target 350)
**Add:**
- Summary paragraph (50-75 tokens)
- Expand each takeaway from 1 sentence to 2-3 sentences (add 100-150 tokens)
- Connection to broader principles (50-75 tokens)

#### Check Your Understanding (Currently 133-204 tokens → Target 350)
**Add:**
- Purpose statement (50-75 tokens)
- Hints/context for each question (10-15 tokens per question)
- Self-study guidance (50-75 tokens)

#### Next Steps (Currently 129-210 tokens → Target 350)
**Add:**
- Lesson completion summary (50-75 tokens)
- Preview of next lesson content (75-100 tokens)
- Optional resources/further reading (50-75 tokens)

### Code-Heavy Sections

#### For sections >600 tokens:
**Option 1: Split into multiple H2 sections**
- Example: "ROS 2 Actions for Long-Running Tasks" (950 tokens)
  - Split into: "Understanding ROS 2 Actions" (400 tokens) + "Implementing Action Clients" (400 tokens)

**Option 2: Move code to H3 subsections**
- Keep explanation at H2 level (300-400 tokens)
- Move code examples to H3 subsections (not counted for RAG chunking)

**Recommendation**: Use Option 1 for better RAG retrieval (code + explanation together in chunks)

---

## Lessons Requiring Most Attention

### lesson-2-rclpy-agents.md (3 sections >850 tokens)
- "Managing Agent State Across Callbacks" (936 tokens) → Split into 2 sections
- "Structuring an AI Agent Node" (886 tokens) → Split into 2 sections
- "Integrating Machine Learning Models" (851 tokens) → Split into 2 sections

### lesson-4-urdf-humanoids.md (3 sections >750 tokens)
- "Building a Humanoid Upper Body" (928 tokens) → Split into 2 sections
- "Understanding URDF XML Structure" (789 tokens) → Trim or split
- "Defining Joints and Kinematics" (756 tokens) → Trim or split

### lesson-4-services-actions.md (3 sections >620 tokens)
- "ROS 2 Actions for Long-Running Tasks" (950 tokens) → Split into 2 sections
- "Understanding ROS 2 Services" (687 tokens) → Trim to 500 tokens
- "When to Use Topics, Services, or Actions" (623 tokens) → Trim to 500 tokens

---

## Alternative Approach: Adjust Requirements

**Consider**: The 300-500 token requirement may be too strict for technical content with code examples.

**Proposal**: Revise constitution to allow:
- Structural sections: 200-400 tokens (more flexible for concise content)
- Instructional sections: 300-600 tokens (accommodate code examples)
- Maximum: 700 tokens (hard limit for any section)

**Rationale**:
- RAG systems can handle variable chunk sizes
- Pedagogical clarity sometimes requires longer explanations
- Code examples are inherently verbose but valuable

**If adopted**: Would reduce failing sections from 65% to ~30%, focusing effort on truly problematic sections.

---

## Success Metrics

**Target after remediation:**
- ≥90% of H2 sections within token range
- All structural sections expanded with meaningful content
- No sections >700 tokens
- RAG retrieval test queries return relevant, complete answers

**Validation:**
- Re-run token counting script
- Test RAG with 10 sample queries across all chapters
- User testing with 2-3 students

---

## Timeline

| Phase | Duration | Deliverable |
|-------|----------|-------------|
| Template Revision | 1-2 hours | Updated section templates |
| Tier 1 Lessons | 4 hours | 4 lessons revised |
| Tier 2 Lessons | 4 hours | 4 lessons revised |
| Tier 3 Lessons | 3 hours | 4 lessons revised |
| Validation | 1 hour | Token validation report |
| **Total** | **13-14 hours** | All 12 lessons compliant |

---

## Recommendation

**Immediate action**: Revise Tier 1 lessons (lesson-1-foundations, lesson-2-nodes, lesson-3-topics-messages, lesson-2-rclpy-agents) to validate the approach and templates.

**Deferred action**: Complete Tier 2 and Tier 3 after user feedback on revised Tier 1 lessons.

**Alternative**: Propose constitution amendment to adjust token requirements based on section type.
