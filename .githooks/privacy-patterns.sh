# 隱私與工具署名的比對規則單一來源，由 pre-commit hook 與 CI 掃描共用。
# 本檔本身含有規則字面值，因此掃描時一律略過 .githooks/ 底下的檔案。

blocked_name_pattern='(co''dex|cl''aude|g''utc)'
blocked_content_pattern='(co''dex|cl''aude|g''utc|co-''authored-by|co-''author)'
email_pattern='[[:alnum:]._%+-]+@[[:alnum:].-]+\.[[:alpha:]]{2,}'
home_path_pattern='/(home|Users)/[^/[:space:]]+'
secret_pattern='(BEGIN (RSA |EC |OPENSSH )?PRIVATE KEY|gh[pousr]_[A-Za-z0-9_]{20,}|sk-[A-Za-z0-9_-]{20,})'
