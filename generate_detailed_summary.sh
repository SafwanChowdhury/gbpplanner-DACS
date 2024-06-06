#!/bin/bash

# Check if branch name and starting commit are provided
if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: $0 <branch-name> <starting-commit>"
    exit 1
fi

branch_name=$1
starting_commit=$2

# Checkout the branch
git checkout $branch_name

# File to store the final summary
output_file="branch_summary_${branch_name}.txt"

# Add branch information
echo "Branch Summary for '$branch_name'" > $output_file
echo "====================================" >> $output_file

# Add commits summary
echo "Commits Summary:" >> $output_file
echo "------------------------------------" >> $output_file
git log $starting_commit..HEAD --pretty=format:"%h - %an, %ar : %s" >> $output_file
echo "" >> $output_file

# Add code changes summary
echo "Code Changes Summary:" >> $output_file
echo "------------------------------------" >> $output_file
git diff $starting_commit..HEAD --stat >> $output_file

# Add detailed code changes
echo "" >> $output_file
echo "Detailed Code Changes:" >> $output_file
echo "------------------------------------" >> $output_file

# Loop through each commit and add detailed changes
for commit in $(git rev-list $starting_commit..HEAD); do
    echo "Commit: $commit" >> $output_file
    echo "------------------------------------" >> $output_file
    git show $commit --stat >> $output_file
    git show $commit --pretty=format:"" >> $output_file
    echo "" >> $output_file
done

# Notify user
echo "Summary has been compiled into $output_file"

