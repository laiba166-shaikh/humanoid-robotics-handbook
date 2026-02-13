import re


def validate_password(password: str) -> tuple[bool, list[str]]:
    """
    Validate password strength per spec FR-002.

    Requirements:
    - Minimum 8 characters
    - At least 1 uppercase letter
    - At least 1 digit
    - At least 1 special character (!@#$%^&*()_+=-)

    Returns:
        (is_valid, error_messages)
    """
    errors = []

    if len(password) < 8:
        errors.append("Password must be at least 8 characters long")

    if not re.search(r'[A-Z]', password):
        errors.append("Password must contain at least one uppercase letter")

    if not re.search(r'\d', password):
        errors.append("Password must contain at least one digit")

    if not re.search(r'[!@#$%^&*()_+\-=\[\]{};:\'",.<>?/\\|`~]', password):
        errors.append("Password must contain at least one special character")

    return len(errors) == 0, errors
