`Init_params/defaults/` contains tracked repository defaults.

`Init_params/local/` contains robot-specific overrides and runtime writes.

Read order is:
- `Init_params/local/...`
- `Init_params/defaults/...`

`Init_params/local/` is ignored by git.

Current robot-specific data imported from local robot snapshot:
- `Init_params/local/Real/Real_params.json`
