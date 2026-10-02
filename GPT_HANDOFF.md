# GPT implementation and Claude continuation

Repository: https://github.com/Ambesnoff/Capstone-Bombsquad-Robot

Status: PR #2 (audit fixes) merges into this branch and PR #1 merges this branch into `main`. After that, start new work from `main` on a new branch.

- GPT implementation branch: **gpt/architecture-v2**.
- Original code branch: **baseline/original-v1**. This is the pre-implementation snapshot; its 44 original software tests pass.
- **main** is unchanged by this task. Its earlier upload was made while implementation files were still in progress, so use the explicit original baseline when continuing the prior v1 work.

To continue original work in Claude without touching the GPT checkout, clone into a different directory:

```sh
git clone --branch baseline/original-v1 https://github.com/Ambesnoff/Capstone-Bombsquad-Robot.git robot-claude-v1
```

To continue this implementation, use a separate checkout of `gpt/architecture-v2`. Keep Claude and GPT on separate checkouts. Do not merge or cherry-pick between them until you intentionally choose those changes. The original archive is also retained at `releases/robot-v1-baseline.zip`.

The mode switch is **SB**, mapped to CH6 in the fast example. SA/CH5 arms, SD/CH7 stops, SC/CH8 selects reverse. Verify actual radio mixes; physical switch names are not fixed channel numbers. Fast modes permit 0.8 A Gentle, 1.5 A Normal, and 2.5 A Boost. No 1.2 A fast-path cap remains. Thermal/budget/protective reductions stay visible.

Read `IMPLEMENTATION_STATUS.md` for delivered code/tests and remaining physical acceptance work, `ROBOT_SETUP_GUIDE.md` for installation, and `CLEANUP.md` for removable generated files. Software implementation and test results do not replace robot measurements.
