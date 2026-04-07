# Claude Code Rules

## File Exclusion: `kr_` prefix

Files whose names begin with `kr_` must be **completely ignored** in all operations unless the user explicitly references the exact file name in their prompt.

**What "ignored" means:**
- Do not read, search, or open these files unless explicitly asked
- Do not include them in directory listings, project analyses, or summaries
- Do not suggest, reference, or mention them during exploration or refactoring
- Do not apply changes to them as a side effect of working on other files

**Exception — explicit call only:**
The rule is lifted only when the user's prompt contains the exact file name (e.g., `kr_something.py`). In that case, treat the file normally for that request only.
