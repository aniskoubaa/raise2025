#!/bin/bash

# RAISE2025 GitHub Update Script
# This script automates the process of updating the repository on GitHub

echo "🚀 Starting RAISE2025 GitHub Update Process..."
echo "================================================="

# Get current timestamp
TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')

# Check if we're in the right directory
if [ ! -f "README.md" ]; then
    echo "❌ Error: README.md not found. Please run this script from the RAISE2025 directory."
    exit 1
fi

# Check if git is initialized
if [ ! -d ".git" ]; then
    echo "❌ Error: Git repository not initialized. Please run 'git init' first."
    exit 1
fi

# Add all changes
echo "📝 Adding all changes to git..."
git add .

# Check if there are changes to commit
if git diff --cached --quiet; then
    echo "ℹ️  No changes to commit."
else
    # Commit with timestamp
    echo "💾 Committing changes..."
    git commit -m "Update repository - $TIMESTAMP"
fi

# Check if remote origin exists
if ! git remote get-url origin > /dev/null 2>&1; then
    echo "❌ Error: No remote origin configured."
    echo "Please add your GitHub repository as remote origin:"
    echo "git remote add origin https://github.com/USERNAME/RAISE2025.git"
    exit 1
fi

# Push to GitHub
echo "🚀 Pushing to GitHub..."
git push origin main

if [ $? -eq 0 ]; then
    echo "✅ Successfully updated GitHub repository!"
    echo "Repository URL: $(git remote get-url origin)"
else
    echo "❌ Error pushing to GitHub. Please check your remote configuration and permissions."
    exit 1
fi

echo "================================================="
echo "✨ GitHub update completed at $TIMESTAMP"
echo "=================================================" 