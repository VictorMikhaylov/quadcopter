#!/usr/bin/env bash
set -e

python3.10 -m venv --clear --copies .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
pip install black isort pipdeptree pre-commit pytest rope