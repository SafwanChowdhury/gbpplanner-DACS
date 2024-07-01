#!/bin/bash

# File to store the final summary
output_file="branch_summary.txt"

# Add branch information
echo "Branch Summary for 'your-branch'" > $output_file
echo "====================================" >> $output_file

# Add commits summary
echo "Commits Summary:" >> $output_file
echo "------------------------------------" >> $output_file
git log --pretty=format:"%h - %an, %ar : %s" >> $output_file
echo "" >> $output_file

# Add code changes summary
echo "Code Changes Summary:" >> $output_file
echo "------------------------------------" >> $output_file
git diff --stat >> $output_file

# Notify user
echo "Summary has been compiled into $output_file"

