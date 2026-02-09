# Lesson Body Template

Every lesson MUST follow this skeleton for consistency and RAG optimization.

## Mandatory Structure

```markdown
# Lesson X.Y: [Title]

**Duration**: XX minutes
**Hardware Tier**: Tier X (Description)
**Layer**: LX (Layer Name)

## Learning Objectives

By the end of this lesson, you will be able to:
- [Bloom's verb] + [measurable outcome]
- [Bloom's verb] + [measurable outcome]
- [Bloom's verb] + [measurable outcome]

## [The Hook — Why This Matters]

[2-3 sentences connecting to a real-world application or problem.
 Answer: "Why should I care about this?"]

## [Core Section 1 — Descriptive H2 Header]

[300-500 tokens. Self-contained for RAG retrieval.
 Descriptive header, not clever — "How Nodes Communicate"
 not "Talking Shop"]

## [Core Section 2 — Descriptive H2 Header]

[Continue pattern. Each H2 = one RAG chunk boundary.]

## [Core Section N — as many as needed]

[Each section 300-500 tokens, keyword-rich header.]

## Key Takeaways

- [Fact 1 — the RAG chatbot retrieves these when asked about this topic]
- [Fact 2]
- [Fact 3]
- [Fact 4 — optional]
- [Fact 5 — optional]

## Check Your Understanding

1. [Factual recall question]
2. [Application question — "Given X, what would you do?"]
3. [Comparison question — "How does X differ from Y?"]
4. [Optional: deeper analysis question]
5. [Optional: scenario-based question]

## Next Steps

[1-2 sentences pointing to the next lesson with context.
 Example: "Now that you understand nodes, the next lesson
 covers how they communicate using topics and messages."]

---
**Hardware Tier X Note**: [Tier-specific alternative or guidance]
```

## Section Rules

| Rule | Rationale |
|------|-----------|
| Each H2 section = 300-500 tokens | Optimal RAG chunk size for retrieval |
| H2 headers are descriptive + keyword-rich | Improves semantic search accuracy |
| No questions as H2 headers | Use statements: "Why Humanoids Need Balance" not "Why Do?" |
| Self-contained sections | Each H2 should make sense if retrieved independently |
| Core sections before Key Takeaways | Build understanding, then summarize |

## Example: Good vs Bad Headers

| Bad | Good |
|-----|------|
| "Getting Started" | "Installing ROS 2 on Ubuntu 22.04" |
| "The Pattern" | "Publisher-Subscriber Pattern in ROS 2" |
| "More Details" | "Configuring Quality of Service for Reliable Messaging" |
| "The Art of Robot Talk" | "How ROS 2 Nodes Communicate with Topics" |

## Key Takeaways Guidelines

- Write as standalone facts that answer common questions
- These are the primary RAG retrieval targets for this lesson
- Each takeaway should be 1-2 sentences max
- Avoid vague summaries — be specific and factual

## Check Your Understanding Guidelines

- Mix factual recall (40%) and application (60%) questions
- Questions should test whether objectives were met
- Include the lesson context needed to answer (do not require outside knowledge)
- Avoid yes/no questions — prefer "explain", "compare", "what would happen if"
