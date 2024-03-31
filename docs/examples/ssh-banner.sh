#!/bin/bash
# Check if we are on SSH
if [ -n "$SSH_CLIENT" ] || [ -n "$SSH_TTY" ]; then
    # ANSI color codes for red and bold
    RED_BOLD="\033[1;31m"
    # Reset color
    RESET="\033[0m"

    # Read the banner content and then print it in red and bold
    while IFS= read -r line
    do
        echo -e "${RED_BOLD}${line}${RESET}"
    done < /etc/ssh/custom_info
fi

