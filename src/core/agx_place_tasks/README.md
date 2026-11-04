# agx_place_tasks

把夹爪里的试管放进**机器人机载架**的指定槽位：配置位姿、无识别、无视觉对准。

- 任务类型：`place_test_tube`
- 参数：`slot`（1..N），可选 `rack_profile`
- 效果：`robot.object_at_holder`（只在测得槽位占用时产生）
- 配置与失败语义：见 `docs/configuration.md`
- 先验证臂能否够到机载架：`docs/reachability_check.md`（只规划不执行）

分层：`agx_place_tasks -> agx_task_workflow, agx_robot_task_runtime`。
本包不依赖 `agx_robot_workflow`，宿主通过 `RegisterPlaceTasks()` 注入端口。

```bash
catkin_make --pkg agx_place_tasks
catkin_make run_tests_agx_place_tasks
```
