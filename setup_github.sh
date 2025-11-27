#!/bin/bash

# GitHub Repository Setup Script
# This script helps you set up your git repository for GitHub

echo "🚀 CARLA ADAS Project - GitHub Setup"
echo "====================================="
echo ""

# Check if git is installed
if ! command -v git &> /dev/null; then
    echo "❌ Error: Git is not installed. Please install git first."
    exit 1
fi

# Check if already a git repository
if [ -d .git ]; then
    echo "⚠️  Git repository already initialized."
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "📦 Initializing git repository..."
    git init
fi

echo ""
echo "📝 Adding files to git..."
git add tara.py
git add README.md
git add requirements.txt
git add .gitignore
git add LICENSE
git add GITHUB_SETUP.md

echo ""
read -p "Add other Python files? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    git add *.py
    git add rss/
fi

echo ""
echo "📋 Files staged. Review with: git status"
echo ""

read -p "Create initial commit? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    git commit -m "Initial commit: CARLA ADAS Simulation System

- Comprehensive ADAS implementation with FCW, AEB, ACC, LDW, BSD, TSR
- Intelligent overtaking system
- Scenario manager for testing
- Real-time sensor visualization
- Professional documentation and setup"
    echo "✅ Initial commit created!"
else
    echo "⏭️  Skipping commit. You can commit later with: git commit -m 'Your message'"
fi

echo ""
echo "====================================="
echo "✅ Setup complete!"
echo ""
echo "Next steps:"
echo "1. Create a repository on GitHub (see GITHUB_SETUP.md for details)"
echo "2. Add remote: git remote add origin https://github.com/YOUR_USERNAME/REPO_NAME.git"
echo "3. Push: git push -u origin main"
echo ""
echo "For detailed instructions, see GITHUB_SETUP.md"
echo ""

