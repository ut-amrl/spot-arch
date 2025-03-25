"""
Run: rosinstall_generator desktop_full --rosdistro noetic --deps --tar > desktop-full.rosinstall
then run this script to generate a bash script to clone all the needed repositories
"""

import re

# Input and output file names
input_file = "desktop-full.rosinstall"
output_file = "clone_repos.sh"

# Patterns to extract local-name, uri, and version
local_name_pattern = re.compile(r"local-name:\s+([\w/-]+)")
version_pattern = re.compile(r"version:\s+([^\s]+)")

with open(input_file, "r") as f:
    lines = f.readlines()

# Initialize an empty list to store git commands
commands = []

# Parse lines to extract information and form git clone commands
for i in range(len(lines)):
    # Extract local-name and version
    local_name_match = local_name_pattern.search(lines[i])
    version_match = version_pattern.search(lines[i + 2]) if (i + 2) < len(lines) else None
    
    if local_name_match and version_match:
        # Get the package name and version
        package_name = local_name_match.group(1).split('/')[0]  # Take the base name from local-name
        version_full = version_match.group(1)
        
        # Extract the version number (e.g., "1.14.0" from "actionlib-release-release-noetic-actionlib-1.14.0-1")
        version = re.search(r"(\d+\.\d+\.\d+)", version_full).group(1)
        
        # Form the git clone command with the standard repository URL
        commands.append(f"git clone https://github.com/ros/{package_name}.git -b {version}\n")

# Remove duplicates and sort the commands
commands = sorted(list(set(commands)))

# Wrap each command to handle failure and store it in a list if it fails
wrapped_commands = [
    "#!/bin/bash\n",
    "failed_clones=()\n\n"
]

for command in commands:
    wrapped_commands.append(
        f"{command.strip()} || failed_clones+=(\"{command.strip()}\")\n"
    )

# Add final output for any failed clone commands
wrapped_commands.extend([
    "\nif [ ${#failed_clones[@]} -ne 0 ]; then\n",
    "  echo 'The following git clone commands failed:'\n",
    "  for cmd in \"${failed_clones[@]}\"; do\n",
    "    echo \"$cmd\"\n",
    "  done\n",
    "  echo 'Please clone them manually.'\n",
    "fi\n"
])

# Write wrapped commands to the output file
with open(output_file, "w") as f:
    f.writelines(wrapped_commands)

print(f"Bash script '{output_file}' generated successfully.")

