# Mechtronic 4 專案

機電整合自走車專案 - Arduino + Raspberry Pi 雙控制器架構。

## Neuromorphic Multi-Agent 系統

> **本專案使用 Neuromorphic Multi-Agent 系統進行任務管理**
>
> 完整協作指南：`~/.claude/neuromorphic/SYSTEM_GUIDE.md`

### 使用規則

**一般任務**：Claude Code 可直接執行，不需派發 agent。

**使用 PFC 系統時**（複雜多步驟任務、用戶明確要求）：

1. **必須透過 Task tool 派發 agent** - Claude Code 是「調度者」，不是「執行者」
2. **完整執行循環**：
   - 派發 `pfc` agent 規劃任務
   - 派發 `executor` agent 執行子任務
   - 派發 `critic` agent 驗證結果
   - 派發 `memory` agent 存經驗
3. **auto-compact 後必須檢查任務進度** - 讀取 DB 恢復狀態

**禁止行為（使用 PFC 時）：**
- ❌ 直接用 Bash 執行本應由 Executor 做的檔案操作/程式碼修改
- ❌ 自己扮演 PFC 規劃而不派發 Task tool
- ❌ 跳過 Critic 驗證直接完成任務

**Agent 限制：**
- ❌ Executor 禁止執行 `git commit` / `git push` - 由 Claude Code 主體審核後提交
- ❌ Agent 不得覆蓋人工編排的文檔，除非明確指示

### 可用 Agents

| Agent | subagent_type | 用途 |
|-------|---------------|------|
| PFC | `pfc` | 任務規劃、協調 |
| Executor | `executor` | 執行單一任務 |
| Critic | `critic` | 驗證結果 |
| Memory | `memory` | 知識管理 |
| Researcher | `researcher` | 資訊收集 |

### 系統入口（供 Agent 使用）

```python
import sys
import os
sys.path.insert(0, os.path.expanduser('~/.claude/neuromorphic'))
from servers.tasks import get_task_progress, create_task
from servers.memory import search_memory, load_checkpoint
```

### 使用方式

對 Claude Code 說：「使用 pfc agent 規劃 [任務描述]」
