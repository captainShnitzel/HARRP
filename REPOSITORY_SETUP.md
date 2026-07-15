# Repository setup

## 1. Configure Git identity

```bash
git config --global user.name "YOUR NAME"
git config --global user.email "YOUR_GITHUB_EMAIL"
```

## 2. Review imported files

```bash
git status
git diff --no-index /dev/null README.md
```

## 3. Create the first commit

```bash
git add .
git commit -m "Initial HARRP V1 repository import"
git tag -a v1.0.0 -m "HARRP V1 bachelor project baseline"
```

## 4. Create a private GitHub repository

With GitHub CLI:

```bash
gh auth login
gh repo create HARRP --private --source=. --remote=origin --push
git push origin v1.0.0
```

Without GitHub CLI, create an empty private repository named `HARRP` on GitHub, then run:

```bash
git remote add origin https://github.com/YOUR_USERNAME/HARRP.git
git push -u origin main
git push origin v1.0.0
```

Do not initialize the GitHub repository with a README, license, or `.gitignore` because those files already exist locally.
