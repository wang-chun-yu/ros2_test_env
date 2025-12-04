#!/usr/bin/env bash

set -euo pipefail

# 本脚本用于导出当前工作空间 src 目录下的仓库信息到 script/source.repos

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SRC_DIR="${WS_DIR}/src"
OUTPUT_FILE="${SCRIPT_DIR}/source.repos"

if ! command -v vcs >/dev/null 2>&1; then
  echo "错误: 未找到 'vcs' 命令，请先在 ROS2 环境中安装/激活 vcstool (vcs)。" >&2
  exit 1
fi

if [ ! -d "${SRC_DIR}" ]; then
  echo "错误: 未找到 src 目录: ${SRC_DIR}" >&2
  exit 1
fi

cd "${WS_DIR}"

echo "导出 ${SRC_DIR} 中的仓库到 ${OUTPUT_FILE}..."

# 先写入顶层键
echo "repositories:" > "${OUTPUT_FILE}"

# 逐个子目录处理，避免因为某个仓库没有上游分支导致整体失败
for entry in "${SRC_DIR}"/*; do
  [ -d "${entry}" ] || continue

  # 如果是 git 仓库（直接在该目录下有 .git）
  if [ -d "${entry}/.git" ]; then
    name="$(basename "${entry}")"

    # 检查是否配置了上游分支
    if git -C "${entry}" rev-parse --abbrev-ref --symbolic-full-name '@{u}' >/dev/null 2>&1; then
      echo "  处理仓库: ${name}"
      # 使用 vcs export 导出该仓库的 .repos 片段，但去掉开头的 'repositories:' 行
      vcs export "${entry}" | sed '1d' >> "${OUTPUT_FILE}"
    else
      echo "  跳过仓库(未配置上游分支): ${name}"
      echo "  # 跳过仓库(未配置上游分支): ${name}" >> "${OUTPUT_FILE}"
    fi
  fi
done

echo "导出完成，结果已写入: ${OUTPUT_FILE}"



