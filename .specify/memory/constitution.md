<!-- Sync Impact Report -->
<!--
Version change: 0.0.0 → 1.0.0
Modified principles:
  - Added: I. Library-First, II. CLI Interface, III. Test-First (NON-NEGOTIABLE), IV. Integration Testing, V. Observability, VI. Simplicity
Added sections: Additional Constraints, Development Workflow
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Book Constitution

## Core Principles

### I. Library-First
Every feature starts as a standalone library. Libraries must be self-contained, independently testable, and documented. A clear purpose is required - no organizational-only libraries.

### II. CLI Interface
Every library exposes functionality via a CLI. Text in/out protocol: stdin/args → stdout, errors → stderr. Support JSON + human-readable formats.

### III. Test-First (NON-NEGOTIABLE)
TDD is mandatory: Tests are written → User approved → Tests fail → Then implement. The Red-Green-Refactor cycle is strictly enforced.

### IV. Integration Testing
Focus areas requiring integration tests: New library contract tests, Contract changes, Inter-service communication, Shared schemas.

### V. Observability
Text I/O ensures debuggability. Structured logging is required.

### VI. Simplicity
Start simple, adhere to YAGNI (You Aren't Gonna Need It) principles.

## Additional Constraints

Technology stack requirements: Python 3.9+, Node.js 18+ for frontend components. All dependencies must be open-source and actively maintained.

## Development Workflow

Code reviews are mandatory for all changes. All code must pass automated tests and linters before merging. Releases follow a semantic versioning scheme.

## Governance

This Constitution supersedes all other practices. Amendments require a documented proposal, review, and approval from core maintainers. All pull requests and code reviews must verify compliance with these principles. Complexity must be justified by clear business value.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
