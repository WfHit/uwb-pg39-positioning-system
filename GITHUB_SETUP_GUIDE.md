# GitHub Repository Setup Guide for UWB PG3.9

## Prerequisites Setup

### 1. Install Git for Windows

1. **Download Git**: Go to https://git-scm.com/download/win
2. **Install Git**: Run the installer with default settings
3. **Verify Installation**: After installation, restart PowerShell and run:
   ```powershell
   git --version
   ```

### 2. Configure Git (First Time Setup)

```powershell
# Set your username and email (use your GitHub credentials)
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Set default branch name to main
git config --global init.defaultBranch main
```

## GitHub Repository Creation

### Option A: Create via GitHub Web Interface (Recommended)

1. **Go to GitHub**: Visit https://github.com
2. **Sign in** to your GitHub account (create one if needed)
3. **Create New Repository**:
   - Click the "+" icon → "New repository"
   - Repository name: `uwb-pg39-positioning-system`
   - Description: `Professional UWB positioning system based on STM32F103CB with DecaWave DWM3000`
   - Make it **Public** (or Private if preferred)
   - ❌ **DO NOT** initialize with README, .gitignore, or license (we already have files)
   - Click "Create repository"

4. **Copy Repository URL**: Copy the HTTPS URL shown (e.g., `https://github.com/yourusername/uwb-pg39-positioning-system.git`)

### Option B: Create via GitHub CLI (Alternative)

If you have GitHub CLI installed:
```powershell
gh repo create uwb-pg39-positioning-system --public --description "Professional UWB positioning system based on STM32F103CB with DecaWave DWM3000"
```

## Initialize Local Git Repository

Run these commands in your project directory (`C:\Users\frank\uwb_pg39_ws\uwb_pg39`):

```powershell
# Initialize git repository
git init

# Create .gitignore file
New-Item -Path .gitignore -ItemType File
```

## Create .gitignore File

Add the following content to `.gitignore`:

```gitignore
# Build directories
build/
Build/
BUILD/

# IDE files
.vscode/
.vs/
*.suo
*.user
*.userosscache
*.sln.docstates

# Compiled files
*.o
*.obj
*.elf
*.hex
*.bin
*.map
*.lst

# Debug files
*.pdb
*.idb
*.ilk

# CMake generated files
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
CTestTestfile.cmake
_deps/

# Keil/uVision files
*.uvopt
*.uvgui*
Listings/
Objects/

# Temporary files
*.tmp
*.temp
*~

# OS generated files
Thumbs.db
.DS_Store
*.swp
*.swo

# Log files
*.log

# Documentation build
docs/_build/
doxygen_output/
```

## Push Code to GitHub

```powershell
# Add all files to git
git add .

# Create initial commit
git commit -m "Initial commit: Professional UWB PG3.9 positioning system

- Complete DecaWave DWM3000 driver integration
- Advanced positioning algorithms (2D/3D, Center Mass, Least Squares, Taylor Series)
- ARM CMSIS DSP optimization for STM32F103CB
- Professional CMake build system with GNU ARM toolchain
- Hardware abstraction layer for flash, OLED, and UWB interfaces
- Snake case naming convention throughout
- Comprehensive documentation and build scripts"

# Add remote repository (replace URL with your actual repository URL)
git remote add origin https://github.com/yourusername/uwb-pg39-positioning-system.git

# Push to GitHub
git push -u origin main
```

## Verification

After pushing, verify your repository:

1. **Visit your GitHub repository** in a web browser
2. **Check that all files are present**:
   - README.md with comprehensive documentation
   - CMakeLists.txt with professional build configuration
   - All source files in proper directory structure
   - ARCHITECTURE_ANALYSIS.md and other documentation

## Repository Features to Add

### 1. Repository Topics/Tags
Add these topics to your GitHub repository (in Settings):
- `uwb`
- `positioning`
- `stm32`
- `decawave`
- `embedded`
- `cmake`
- `arm-cortex-m3`
- `real-time-locating`

### 2. Create Release
After pushing, create your first release:
1. Go to "Releases" tab in your repository
2. Click "Create a new release"
3. Tag: `v1.0.0`
4. Title: `UWB PG3.9 v1.0.0 - Professional Positioning System`
5. Description: Include key features and capabilities

### 3. Enable GitHub Features
- **Issues**: For bug tracking and feature requests
- **Wiki**: For extended documentation
- **Projects**: For development planning

## Quick Commands Reference

```powershell
# Check repository status
git status

# Add specific files
git add filename.c

# Commit changes
git commit -m "Description of changes"

# Push changes
git push

# Pull latest changes
git pull

# Create new branch
git checkout -b feature-branch-name

# Switch branches
git checkout main
```

## Troubleshooting

### If you get authentication errors:
1. **Personal Access Token**: Create a Personal Access Token in GitHub Settings → Developer settings → Personal access tokens
2. **Use token as password** when prompted for credentials

### If you get permission errors:
- Make sure you're the owner of the repository or have push permissions
- Check that the remote URL is correct: `git remote -v`

## Next Steps

After successfully pushing to GitHub:

1. **Share the repository** with your team or community
2. **Set up CI/CD** with GitHub Actions for automated building
3. **Create documentation wiki** for detailed technical specifications
4. **Add issue templates** for bug reports and feature requests
5. **Set up automated testing** for code quality assurance

Your professional UWB positioning system will now be available on GitHub for collaboration and version control!
