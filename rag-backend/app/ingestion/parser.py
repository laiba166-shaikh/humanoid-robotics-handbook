import logging
import re
from pathlib import Path

import yaml

from app.models.domain import LessonFrontmatter

logger = logging.getLogger(__name__)

FRONTMATTER_PATTERN = re.compile(r"^---\s*\n(.*?)\n---\s*\n", re.DOTALL)

REQUIRED_FIELDS = {
    "id", "title", "sidebar_position", "sidebar_label", "description",
    "duration_minutes", "proficiency_level", "layer", "hardware_tier",
    "tier_1_path", "learning_objectives", "keywords", "prerequisites",
    "chapter", "module",
}


def parse_lesson_file(
    path: Path,
) -> tuple[LessonFrontmatter, list[dict]] | None:
    """Parse a lesson markdown file into frontmatter and H2 sections.

    Returns None if the file has invalid/missing frontmatter.
    Each section dict has keys: title (str), content (str).
    """
    text = path.read_text(encoding="utf-8")

    fm_match = FRONTMATTER_PATTERN.match(text)
    if not fm_match:
        logger.warning("No frontmatter found in %s", path)
        return None

    try:
        raw_fm = yaml.safe_load(fm_match.group(1))
    except yaml.YAMLError as e:
        logger.warning("Invalid YAML frontmatter in %s: %s", path, e)
        return None

    if not isinstance(raw_fm, dict):
        logger.warning("Frontmatter is not a dict in %s", path)
        return None

    missing = REQUIRED_FIELDS - set(raw_fm.keys())
    if missing:
        logger.warning("Missing frontmatter fields in %s: %s", path, missing)
        return None

    try:
        frontmatter = LessonFrontmatter(**raw_fm)
    except Exception as e:
        logger.warning("Frontmatter validation failed in %s: %s", path, e)
        return None

    body = text[fm_match.end():]

    sections = _split_h2_sections(body)

    return frontmatter, sections


def _split_h2_sections(body: str) -> list[dict]:
    """Split markdown body at H2 (##) boundaries.

    Returns list of {title: str, content: str}.
    If no H2 headers, returns entire body as a single section.
    """
    h2_pattern = re.compile(r"^## (.+)$", re.MULTILINE)
    matches = list(h2_pattern.finditer(body))

    if not matches:
        stripped = body.strip()
        if stripped:
            return [{"title": "Content", "content": stripped}]
        return []

    sections = []
    for i, match in enumerate(matches):
        title = match.group(1).strip()
        start = match.end()
        end = matches[i + 1].start() if i + 1 < len(matches) else len(body)
        content = body[start:end].strip()
        if content:
            sections.append({"title": title, "content": content})

    return sections
