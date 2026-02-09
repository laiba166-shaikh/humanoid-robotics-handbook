# Writing Standards

Standards for every lesson, chapter overview, and module page. These ensure students, career-switchers, instructors, and industry reviewers receive consistent, high-quality technical content.

## Table of Contents

- [Stakeholder Alignment](#stakeholder-alignment)
- [Core Principles](#core-principles)
- [Language Rules](#language-rules)
- [Banned Phrases](#banned-phrases)
- [Technical Terminology Protocol](#technical-terminology-protocol)
- [Engagement Patterns](#engagement-patterns)
- [Voice and Tone](#voice-and-tone)

## Stakeholder Alignment

| Stakeholder | Need | How to Serve |
|-------------|------|-------------|
| Beginners | Clear foundations, no assumed domain knowledge | Concept-first lessons, analogies to software they know |
| Career switchers | Fast path from existing skills | Python-first examples, map to patterns they know |
| Bootcamp participants | Structured, time-boxed learning | Duration estimates, clear prerequisites, self-assessment |
| ESL readers | Parseable English, no idioms | Short sentences, defined terms, consistent vocabulary |
| Instructors | Teachable structure | Learning objectives, check-your-understanding, tier paths |
| Industry reviewers | Technical accuracy | Verified code, cited sources, current data |

## Core Principles

### 1. Clarity Is Non-Negotiable

Every sentence must be understood on the first read. If a reader re-reads a sentence, the sentence has failed.

| Rule | Rationale |
|------|-----------|
| One idea per sentence | Reduces cognitive load |
| One concept per paragraph | Allows scanning and retrieval |
| Define before you use | No forward-referencing jargon |
| Simplest correct word | "Send" not "transmit", "use" not "leverage" |

```
BAD:  "The node leverages the pub-sub paradigm to facilitate
       inter-process communication via DDS middleware."

GOOD: "Nodes send messages to each other using publishers and
       subscribers. Under the hood, ROS 2 uses DDS (Data
       Distribution Service) to deliver these messages."
```

### 2. Concrete Before Abstract

Lead with something tangible — a code snippet, a real-world analogy, or a diagram — before explaining theory.

```
BAD:  "A ROS 2 topic is a named bus over which nodes exchange
       typed messages using an anonymous publish-subscribe model."

GOOD: "Imagine a radio channel. Anyone can broadcast on it, and
       anyone tuned in will hear the message. A ROS 2 topic works
       the same way — it's a named channel where nodes publish
       and subscribe to messages."
```

**Pattern**: Example/Analogy -> Definition -> Details -> "Why It Matters"

### 3. Respect the Reader's Time

- Get to the point in the first 2 sentences of every section
- Use bullet points and tables for scannable content
- **Bold** key terms on first use
- Provide duration estimates so readers can plan study sessions
- Never pad content — if 3 sentences suffice, do not write 6

### 4. Progressive Disclosure

Teach the 80% case first. Edge cases and advanced patterns come later or link to separate sections.

| Stage | Include | Defer |
|-------|---------|-------|
| First mention | Core behavior, basic usage | Configuration options, internals |
| Code example | Working minimum | Error handling, logging |
| Explanation | "What it does" and "why" | "How it works internally" |

### 5. Every Concept Earns Its Place

If a concept does not connect to something the reader will build, question whether it belongs. Within 2 paragraphs of introducing any concept, answer: **"Why does this matter to you?"**

## Language Rules

### Sentence Structure
- **Target**: 15-20 words per sentence
- **Maximum**: 30 words — break longer sentences
- **Paragraphs**: 3-5 sentences maximum
- **Active voice always**: "The publisher sends a message" not "A message is sent by the publisher"
- **Transition words**: "First," "However," "As a result," "For example," to guide the reader

### Section Header Rules (Critical for RAG)

Section headers directly impact RAG retrieval quality:

| Rule | Example |
|------|---------|
| Descriptive, not clever | "How ROS 2 Nodes Communicate" not "The Art of Robot Talk" |
| Self-contained meaning | "Installing ROS 2 on Ubuntu" not "Getting Started" |
| Include key terms | "Publisher-Subscriber Pattern in ROS 2" not "The Pattern" |
| No questions as H2 | "Why Humanoids Need Balance" not "Why Do Humanoids Need Balance?" |

## Banned Phrases

These phrases undermine the reader's confidence. Remove them entirely.

| Never Write | Why | Write Instead |
|-------------|-----|---------------|
| "Simply do X" | Nothing is simple to a beginner | "Do X" |
| "Obviously" | If it were obvious, you would not explain it | Remove or explain why |
| "Just" | Minimizes complexity | Remove entirely |
| "Easy" / "Trivial" | Shames readers who struggle | "Straightforward" or remove |
| "As everyone knows" | Excludes those who do not know | State the fact directly |
| "He/she" | Not inclusive | "They" or "the developer" |

## Technical Terminology Protocol

1. **First use**: Write the full term with definition in bold — **Node**: an independent ROS 2 executable that performs computation
2. **Subsequent uses**: Use the short form consistently — "node"
3. **Acronyms**: Expand on first use — "Robot Operating System 2 (ROS 2)" then "ROS 2"
4. **Cross-module terms**: Must match the project glossary exactly — no synonyms

## Engagement Patterns

Use admonitions consistently (Docusaurus syntax shown; adapt to your platform):

| Admonition | When to Use | Example |
|------------|-------------|---------|
| `:::tip` | Efficiency gains, shortcuts | "Use `ros2 topic echo` to debug messages in real-time" |
| `:::note` | Important but non-blocking context | "ROS 2 Iron is the latest release but Humble has longer support" |
| `:::warning` | Common mistakes | "Forgetting to source your workspace is the #1 beginner error" |
| `:::danger` | Physical safety (motors, hardware) | "Always set joint velocity limits before running on hardware" |

**Interactive prompts** within lessons:
- **Try This** (5 min): Mini-exercise within the lesson flow
- **Challenge** (optional): Harder exercise for advanced learners
- **Real-World Connection**: Industry application of the concept

## Voice and Tone

- Technical but accessible
- Encouraging without being patronizing
- Safety-conscious without being alarmist
- Practical over theoretical
- Active, direct sentences
- Second person ("you") for instructions, third person for concepts
