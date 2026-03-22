#!/usr/bin/env bash
# Authenticate gh CLI with a token without leaking it to bash history.
# Usage: source scripts/gh-auth.sh   (or: bash scripts/gh-auth.sh)
set -euo pipefail

read -rsp "Paste GitHub token (input hidden): " token
echo

if [[ -z "$token" ]]; then
  echo "Error: empty token" >&2
  exit 1
fi

echo "$token" | gh auth login --with-token
unset token

echo "Done. Logged in as: $(gh api user -q .login)"
