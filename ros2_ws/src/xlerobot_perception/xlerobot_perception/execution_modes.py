"""Shared read-only capability execution-mode validation."""


def validate_dry_run_mode(mode: str) -> None:
    """Reject ambiguous read-only execution policy at startup."""
    if mode not in ('contract_only', 'observe'):
        raise ValueError("dry_run_mode must be 'contract_only' or 'observe'")
