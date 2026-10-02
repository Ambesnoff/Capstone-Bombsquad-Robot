# GPT implementation and Claude continuation

Repository: https://github.com/Ambesnoff/Capstone-Bombsquad-Robot

Status: PR #2 (audit fixes) and PR #1 (this implementation) are merged into `main`, and `gpt/architecture-v2` is deleted. Start new work from `main` on a new branch.

- GPT implementation branch: **gpt/architecture-v2** (merged into `main`, then deleted).
- Original code branch: **baseline/original-v1**. This is the pre-implementation snapshot; its 44 original software tests pass.
- **main**: v2 with the audit fixes. Use the original baseline above when continuing the prior v1 work.

To continue original work in Claude without touching the GPT checkout, clone into a different directory:

```sh
git clone --branch baseline/original-v1 https://github.com/Ambesnoff/Capstone-Bombsquad-Robot.git robot-claude-v1
```

To continue this implementation, branch from `main`. Keep Claude and GPT on separate checkouts. The local ChatGPT project folder is still on the deleted `gpt/architecture-v2` branch with the pre-audit code; switch it to `main` and pull before working there.

The mode switch is **SB**, mapped to CH6 in the fast example. SA/CH5 arms, SD/CH7 stops, SC/CH8 selects reverse. Verify actual radio mixes; physical switch names are not fixed channel numbers. Fast modes permit 0.8 A Gentle, 1.5 A Normal, and 2.5 A Boost. No 1.2 A fast-path cap remains. Thermal/budget/protective reductions stay visible.

Read `IMPLEMENTATION_STATUS.md` for delivered code/tests and remaining physical acceptance work, and `ROBOT_SETUP_GUIDE.md` for installation. Software implementation and test results do not replace robot measurements.
