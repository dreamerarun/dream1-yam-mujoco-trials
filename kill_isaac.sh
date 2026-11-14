#!/usr/bin/env bash
# Kills leftover Isaac Sim runs and bridges. Its own command line does not contain the pattern,
# so it never matches (and kills) the shell that calls it.
PAT="${1:-run_dream1_isaac}"
for p in $(ps -eo pid=,cmd= | grep -F -- "$PAT" | grep -v grep | awk '{print $1}'); do
  [ "$p" = "$$" ] && continue
  [ "$p" = "$PPID" ] && continue
  kill "$p" 2>/dev/null
done
sleep 1
ps -eo pid=,cmd= | grep -F -- "$PAT" | grep -v grep | wc -l
