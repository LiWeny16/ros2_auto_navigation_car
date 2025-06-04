# Git分支管理指南

## 分支策略

### 主要分支
- `main`: 主分支，始终保持稳定状态，用于生产环境
- `dev`: 开发分支，用于日常开发工作
- `feature/*`: 功能分支，用于开发特定功能
- `hotfix/*`: 热修复分支，用于紧急修复

## 日常开发工作流

### 1. 创建和切换分支

```bash
# 创建并切换到dev分支
git checkout -b dev

# 从dev创建功能分支
git checkout -b feature/carla-integration

# 切换到已存在的分支
git checkout dev

# 查看所有分支
git branch -a
```

### 2. 提交代码

```bash
# 查看文件状态
git status

# 添加文件到暂存区
git add .                    # 添加所有文件
git add specific_file.cpp    # 添加特定文件

# 提交代码
git commit -m "feat: 添加CARLA集成功能"

# 推送到远程分支
git push origin dev
```

### 3. 合并分支

```bash
# 切换到目标分支
git checkout main

# 合并dev分支到main
git merge dev

# 或者使用squash合并（推荐）
git merge --squash dev
git commit -m "合并dev分支的功能"
```

## 回退操作

### 1. 撤销工作区修改（未add）

```bash
# 撤销单个文件的修改
git checkout -- filename

# 撤销所有修改
git checkout -- .
```

### 2. 撤销暂存区修改（已add，未commit）

```bash
# 取消暂存单个文件
git reset filename

# 取消暂存所有文件
git reset
```

### 3. 撤销提交（已commit）

```bash
# 撤销最后一次提交，保留修改
git reset --soft HEAD~1

# 撤销最后一次提交，删除修改
git reset --hard HEAD~1

# 撤销多次提交
git reset --hard HEAD~3  # 撤销最近3次提交
```

### 4. 回退到特定提交

```bash
# 查看提交历史
git log --oneline

# 回退到特定提交（保留修改）
git reset --soft commit_hash

# 回退到特定提交（删除修改）
git reset --hard commit_hash
```

## 常用命令

### 查看状态和历史

```bash
# 查看当前状态
git status

# 查看提交历史
git log --oneline --graph

# 查看分支状态
git branch -v

# 查看远程分支
git branch -r
```

### 分支操作

```bash
# 删除本地分支
git branch -d branch_name

# 强制删除本地分支
git branch -D branch_name

# 删除远程分支
git push origin --delete branch_name

# 重命名分支
git branch -m old_name new_name
```

### 同步操作

```bash
# 拉取远程更新
git pull origin dev

# 获取远程分支信息
git fetch origin

# 推送新分支到远程
git push -u origin dev
```

## 最佳实践

### 1. 提交消息规范

```
feat: 新功能
fix: 修复bug
docs: 文档更新
style: 代码格式化
refactor: 重构
test: 测试相关
chore: 构建过程或辅助工具的变动
```

### 2. 分支命名规范

```
feature/功能名称        # feature/carla-integration
bugfix/bug描述         # bugfix/fix-compilation-error
hotfix/紧急修复描述    # hotfix/security-patch
```

### 3. 开发流程

1. 从main创建dev分支
2. 在dev分支上进行日常开发
3. 对于大功能，从dev创建feature分支
4. 功能完成后，合并回dev
5. dev稳定后，合并回main
6. 定期从main同步更新到dev

### 4. 安全检查

```bash
# 在合并前检查差异
git diff main dev

# 在合并前检查冲突
git merge --no-commit --no-ff dev
git merge --abort  # 如果有问题就取消
```

## 紧急情况处理

### 误删文件恢复

```bash
# 恢复被删除的文件
git checkout HEAD -- deleted_file.cpp
```

### 找回丢失的提交

```bash
# 查看所有操作历史
git reflog

# 恢复到指定操作
git reset --hard HEAD@{2}
```

### 解决合并冲突

```bash
# 查看冲突文件
git status

# 手动编辑冲突文件，然后
git add conflicted_file.cpp
git commit -m "解决合并冲突"
```
