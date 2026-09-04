# SimGrasp3D Lab 協作規範

本專案用於公開學習與技術交流。所有提交都應能由不具備作者本機環境的其他人理解、重現與檢查。

## 隱私與 attribution

- 不提交姓名、私人電子郵件、帳號、主機名稱、家目錄、內網位址、裝置序號或其他可識別個人的資訊。
- 不提交 API key、token、cookie、SSH key、雲端憑證、機器人憑證或相機序號。
- 不加入 AI 助理、編輯器或內容產生工具的署名與共著標記。
- commit、issue、pull request 與文件只描述技術變更，不記錄私人對話或本機工作流程。
- 範例設定使用明顯的假資料；本機設定放在忽略路徑，必要時另提供不含敏感值的 `.example` 檔案。

Repository 已提供 staged-content 與 commit-message hooks。Clone 後執行：

```bash
git config core.hooksPath .githooks
```

兩個 hook 與 CI 共用 `.githooks/privacy-patterns.sh` 的規則。要一次檢查全部版控檔案：

```bash
.githooks/scan-tracked.sh
```

## 模擬結果規範

- 所有本專案產生的結果應標示模擬器、場景版本、random seed、模型版本及主要物理參數。
- 成功率必須附上試驗數、成功定義與失敗分類。
- 不把模擬結果描述成實機成果，也不以外部論文結果冒充本專案重現結果。
- 大型 dataset、checkpoint、ROS bag 與 simulator cache 不直接提交；使用資料下載腳本、checksum、Git LFS 或 release artifact。

## Commit 格式

使用簡潔、可追蹤的命令式訊息：

```text
docs: 補充 RGB-D 模擬限制
feat: 加入點雲抓取候選過濾
test: 新增固定種子的碰撞回歸測試
```

初期建議類型：`docs`、`feat`、`fix`、`test`、`refactor`、`build`、`chore`。

