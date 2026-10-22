# 本机部署状态（2026-10-17 登记）

这份工作区就是**当前真机使用的部署**。它不是某个发布包的完整还原，下面是它的真实来历。

## 来源

| 项 | 值 |
| --- | --- |
| 基线包 | `agilex_ws-source-ee90218.tar.gz`（10-14 21:23 出包，`DEPLOYMENT_REVISION` = `ee90218`） |
| 本机增量 | 2026-10-15 ~ 10-16 的 14 个文件改动（见 `src/tasks/doc/onboard_changes.md`） |
| 增量补丁 | `~/ranger_arm_deploy/onboard-20261016.patch`（相对 ee90218，`patch -p2`，已验证可精确复现本树） |
| 构建 | `catkin_make -DCMAKE_BUILD_TYPE=Release`，产物时间 10-16 12:49 |
| 源码快照 | `~/ranger_arm_deploy/snapshots/agilex_ws-onboard-*.tar.gz` + `SHA256SUMS` |

`DEPLOYMENT_REVISION` 里仍是 `ee90218`，**它只代表基线，不包含本机增量**——不要拿它当这棵树的版本号。

## 和仓库的关系

- 10-13 迁移、10-14 的 20 个文件构建修复已随 `ee90218` 出包，按 WORKLOG 记载已回传仓库。
- 10-15 / 10-16 的 14 个文件**尚未合并进仓库**，目前只存在于本机与上面的补丁/快照里。
- 本机 `git` 仓库（2026-10-17 建立）只用于本机回溯，不是远端仓库的分支。

## 重建方式

任选其一：

```bash
# A. 用快照（最省事）
tar -xzf snapshots/agilex_ws-onboard-20261017-2103.tar.gz -C <目标目录>
source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release

# B. 用基线包 + 补丁
tar -xzf agilex_ws-source-ee90218.tar.gz -C <目标目录>
cd <目标目录>/agilex_ws && patch -p2 < ../../onboard-20261016.patch
source /opt/ros/noetic/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release
```

## 何时不要用这份

- 需要"可追溯的发布版本"时：本树没有对应提交，改动还没进仓库。
- 需要回滚到更早状态时：唯一锚点是 `ee90218` 包（缺 10-15/16 的改动）。
