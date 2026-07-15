# Contributing

## Branches

Use focused branches:

- `feature/<name>` for new capabilities
- `fix/<name>` for corrections
- `docs/<name>` for documentation-only work
- `experiment/<name>` for temporary research work

## Commits

Keep commits small and describe the affected subsystem, for example:

- `firmware: add stale-packet motor inhibit`
- `hand: correct motor 4 polarity`
- `docs: document RS-485 packet timing`
- `hardware: revise shoulder cross-bolt drawing`

## Pull-request checklist

- Hardware target identified
- Pin assignments checked
- Safety behavior reviewed
- Bench test performed with controlled power
- Documentation updated
- Generated files and credentials excluded
