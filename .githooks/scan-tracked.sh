#!/usr/bin/env bash
# 掃描全部版控檔案，讓 pre-commit 的隱私規則也有伺服器端把關。
# 掃全樹而非只掃 diff，可以連同已合併的內容一起檢查，也不需要抓取 base ref。
set -euo pipefail

# shellcheck source=.githooks/privacy-patterns.sh
source "$(dirname "$0")/privacy-patterns.sh"

failed=0

while IFS= read -r -d '' file; do
  if [[ "$file" =~ $blocked_name_pattern ]]; then
    printf '檔名包含不應公開的個人或工具識別字：%s\n' "$file" >&2
    failed=1
  fi

  # 規則本身就寫在 .githooks/ 裡，因此略過該目錄。
  if [[ "$file" == .githooks/* ]]; then
    continue
  fi

  if grep -Eiq "$blocked_content_pattern" -- "$file"; then
    printf '內容包含不應公開的個人、工具或共著識別字：%s\n' "$file" >&2
    failed=1
  fi
  if grep -Eiq "$email_pattern|$home_path_pattern|$secret_pattern" -- "$file"; then
    printf '內容疑似包含電子郵件、本機家目錄或憑證：%s\n' "$file" >&2
    failed=1
  fi
done < <(git ls-files -z)

if ((failed)); then
  printf '請移除敏感內容，或將只供本機使用的檔案加入 .gitignore。\n' >&2
  exit 1
fi

printf '隱私掃描通過。\n'
