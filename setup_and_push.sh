#!/bin/bash
#
# Git 工作流程輔助腳本
#
# 這個腳本提供了一個標準化的 Git 操作流程，用於提交和推送變更。
#
# 使用方法:
# 1. 首次設定:
#    - 手動執行 `git init`
#    - 手動執行 `git remote add origin <URL>`
#
# 2. 日常開發:
#    - 執行此腳本: ./setup_and_push.sh "您的提交訊息"
#

# --- 腳本設定 ---
# 當任何指令失敗時，立即停止腳本執行
set -e

# --- 參數檢查 ---
if [ -z "$1" ]; then
  echo "錯誤: 請提供提交訊息作為第一個參數。"
  echo "用法: $0 \"您的更新說明\""
  exit 1
fi

COMMIT_MESSAGE="$1"
BRANCH_NAME="main"

# --- Git 操作 ---
echo ">> 正在將所有變更加入暫存區..."
git add .

echo ">> 正在提交變更..."
git commit -m "$COMMIT_MESSAGE"

echo ">> 正在從遠端拉取最新變更以進行同步..."
# 'git pull' 是 'git fetch' + 'git merge' 的組合
# --rebase 會將您的本地提交放在遠端變更之上，保持歷史紀錄線性、乾淨
git pull --rebase origin "$BRANCH_NAME"

echo ">> 正在將變更推送到遠端倉庫..."
# -u 參數會設定本地分支追蹤遠端分支，只需在第一次推送時使用
git push -u origin "$BRANCH_NAME"

echo "✅ 操作成功完成！"