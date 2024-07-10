# Git——本地分支落后于远端如何解

- [Git——本地分支落后于远端如何解](#git本地分支落后于远端如何解)
  - [问题现象](#问题现象)
  - [问题表述](#问题表述)
  - [问题解决](#问题解决)
    - [解决方法——子问题1](#解决方法子问题1)
      - [1. 选择合并策略并再次尝试拉取更改](#1-选择合并策略并再次尝试拉取更改)
      - [2. 处理合并或变基后可能的冲突](#2-处理合并或变基后可能的冲突)
      - [3. 推送更改到远程仓库](#3-推送更改到远程仓库)
      - [选择默认行为](#选择默认行为)
    - [解决方法——子问题2](#解决方法子问题2)
      - [1. 手动合并](#1-手动合并)
      - [2. 变基（Rebase）](#2-变基rebase)
      - [3. 强制推送（谨慎使用）](#3-强制推送谨慎使用)
    - [总结](#总结)


## 问题现象

```bash
➜  Code git:(dev) git push
To github.com:Takeung/Learning.git
 ! [rejected]        dev -> dev (non-fast-forward)
error: failed to push some refs to 'github.com:Takeung/Learning.git'
hint: Updates were rejected because the tip of your current branch is behind
hint: its remote counterpart. Integrate the remote changes (e.g.
hint: 'git pull ...') before pushing again.
hint: See the 'Note about fast-forwards' in 'git push --help' for details.
```

## 问题表述

1、在浏览git仓库时临时发现了一个错别字，便直接在git页面通过可视化工具手动修改了；
2、但在之后更新本地分支时并未重新拉取代码，修改后直接提交导致冲突。

## 问题解决

出现这个错误是因为本地 `dev` 分支比远程的 `dev` 分支落后。为了解决这个问题，需要在推送之前将远程的更改合并到本地分支。可以按照以下步骤操作：

1. 拉取远程分支的更改：

    ```bash
    git pull origin dev
    ```

2. 解决冲突（如果有）：

    ```bash
    # 编辑冲突文件，解决冲突
    git add <conflicted-file>
    ```

3. 提交合并后的更改：

    ```bash
    git commit
    ```

4. 推送到远程仓库：

    ```bash
    git push origin dev
    ```

但初步操作之后还是遇到问题：

```bash
➜  Code git:(dev) git pull origin dev
From github.com:Takeung/Learning
 * branch            dev        -> FETCH_HEAD
hint: You have divergent branches and need to specify how to reconcile them.
hint: You can do so by running one of the following commands sometime before
hint: your next pull:
hint: 
hint:   git config pull.rebase false  # merge (the default strategy)
hint:   git config pull.rebase true   # rebase
hint:   git config pull.ff only       # fast-forward only
hint: 
hint: You can replace "git config" with "git config --global" to set a default
hint: preference for all repositories. You can also pass --rebase, --no-rebase,
hint: or --ff-only on the command line to override the configured default per
hint: invocation.
fatal: Need to specify how to reconcile divergent branches.
```

### 解决方法——子问题1

这里遇到的错误提示表明本地分支和远程分支有分歧，需要明确如何处理这些分歧。可以选择使用**合并**、**变基**或**仅允许快进合并**。以下是如何处理这些选项的步骤：

#### 1. 选择合并策略并再次尝试拉取更改

**a. 使用合并（默认策略）**

```bash
git config pull.rebase false
```

**b. 使用变基**

```bash
git config pull.rebase true
```

**c. 仅允许快进合并**

```bash
git config pull.ff only
```

然后再次尝试拉取更改：

```bash
git pull origin dev
```

#### 2. 处理合并或变基后可能的冲突

如果在拉取过程中遇到冲突，需要手动解决这些冲突。Git 会在冲突的文件中标记冲突部分。可以通过以下步骤解决冲突：

1. **打开冲突的文件手动修改以解决冲突。**
2. **标记冲突已解决并添加更改：**

    ```bash
    git add <conflicted-file>
    ```

3. **提交合并或变基：**

    ```bash
    git commit
    ```

#### 3. 推送更改到远程仓库

合并或变基完成后，你可以推送本地更改到远程仓库：

```bash
git push origin dev
```

#### 选择默认行为

如果希望设置默认行为以便以后都使用相同的策略，可以使用 `--global` 参数进行全局配置：

- **默认合并（默认策略）：**

    ```bash
    git config --global pull.rebase false
    ```

- **默认变基：**

    ```bash
    git config --global pull.rebase true
    ```

- **仅允许快进合并：**

    ```bash
    git config --global pull.ff only
    ```

> 在 Git 中，合并（merge）、变基（rebase）和快进合并（fast-forward）是处理分支和合并代码的不同策略。下面是它们的定义和区别：
>
> #### 1. 合并（Merge）
>
> **合并**是一种将两个分支的历史记录结合起来的操作。它会将两个分支的更改整合到一起，并创建一个新的合并提交（merge commit），以表示这两个分支的历史在此点上合并了。
>
> - **优点：**
>   - 保留了完整的历史记录，能清楚地看到各个分支的更改历史。
>   - 能够处理复杂的合并情况。
>
> - **缺点：**
>   - 合并提交可能会使历史记录变得复杂，尤其是有多个合并的情况下。
>   - 在合并提交中，可能会引入额外的噪音，增加历史记录的复杂性。
>
> **示例：**
> ```bash
> git checkout dev
> git pull origin dev
> ```
>
> 如果有新的提交在远程分支上，会合并这些提交到本地分支，并创建一个合并提交。
>
> #### 2. 变基（Rebase）
>
> **变基**是一种将一个分支的提交应用到另一个分支的末端的方法。它将一个分支的提交“重新播放”到另一个分支上，改变了提交的历史记录，使其看起来像是在目标分支的最新提交上创建的一样。
>
> - **优点：**
>   - 使得历史记录更加线性，易于理解。
>   - 可以避免复杂的合并提交，使提交历史更简洁。
>
> - **缺点：**
>   - 会改变提交的历史记录，因此在公共分支上使用时需要小心，特别是当其他人已经基于这些提交进行了工作时。
>   - 可能会引发冲突，需要手动解决。
>
> **示例：**
> ```bash
> git checkout dev
> git pull --rebase origin dev
> ```
>
> 这会将远程 `dev` 分支的提交“重新播放”到本地分支上，使得本地分支看起来像是基于远程分支的最新提交创建的。
>
> #### 3. 快进合并（Fast-Forward）
>
> **快进合并**是一种特殊的合并情况，它不会创建新的合并提交。如果目标分支是当前分支的直接祖先（即当前分支的提交都可以直接包含在目标分支中），则可以进行快进合并。这会将当前分支的指针“快进”到目标分支的末端。
>
> - **优点：**
>   - 不会创建额外的合并提交，历史记录保持线性和简洁。
>   - 操作简单，执行快速。
>
> - **缺点：**
>   - 不适用于需要合并多个分支的情况，因为它只在目标分支是当前分支的直接祖先时才适用。
>   - 可能无法清楚地表示合并的实际过程。
>
> **示例：**
> ```bash
> git checkout dev
> git pull origin dev
> ```
>
> 如果远程 `dev` 分支是本地 `dev` 分支的祖先，则会进行快进合并，使得本地分支直接更新为远程分支的状态。
>
> #### 总结
>
> - **合并（Merge）：** 将两个分支的更改整合到一起，创建一个新的合并提交，保留完整的历史记录。
> - **变基（Rebase）：** 将一个分支的提交“重新播放”到另一个分支的末端，使历史记录更线性，但会改变提交历史。
> - **快进合并（Fast-Forward）：** 在目标分支是当前分支的直接祖先时，直接将当前分支的指针更新到目标分支的末端，不创建新的合并提交。
>

当选择快速合并策略之后，仍然遇到问题

```bash
➜  Code git:(dev) ✗ git config pull.ff only

➜  Code git:(dev) ✗ git pull origin dev

From github.com:Takeung/Learning
 * branch            dev        -> FETCH_HEAD
fatal: Not possible to fast-forward, aborting.
```

遇到 "fatal: Not possible to fast-forward, aborting." 错误时，说明 Git 尝试执行快进合并（fast-forward merge），但当前分支的历史记录与远程分支不兼容。也就是说，当前分支的提交不能简单地移动到远程分支的末端，因为有新的提交在本地或远程分支中，这导致快进合并无法完成。

### 解决方法——子问题2

可以尝试以下几种方法来解决这个问题：

#### 1. 手动合并

如果希望手动合并分支，可以使用普通的合并（merge）策略。这会创建一个新的合并提交来整合分支：

```bash
git pull origin dev --no-ff
```

这将会进行合并而不是快进合并，即使它不是快进合并也会创建一个合并提交。

#### 2. 变基（Rebase）

变基可以将你的本地提交“重新播放”到远程分支的末端，这样可以保持提交历史线性，并且可能会让后续的快进合并变得可行：

```bash
git fetch origin
git rebase origin/dev
```

这将会把本地分支的提交变基到远程 `dev` 分支上。如果变基过程中发生冲突，你需要解决这些冲突并继续变基：

```bash
git rebase --continue
```

完成变基后，你可以进行推送操作（如果需要）：

```bash
git push origin dev
```

#### 3. 强制推送（谨慎使用）

如果你确定本地分支的内容应该覆盖远程分支的内容（这可能会导致远程分支的丢失），你可以使用强制推送。请谨慎使用，因为这会覆盖远程分支上的内容：

```bash
git push origin dev --force
```

**注意：** 强制推送会覆盖远程分支上的提交，可能会影响其他团队成员，因此使用前务必确认。

### 总结

- **合并（Merge）：** `git pull origin dev --no-ff` 进行合并操作。
- **变基（Rebase）：** `git rebase origin/dev` 将你的提交变基到远程分支。
- **强制推送（Force Push）：** `git push origin dev --force` 只在确切需要时使用，可能会覆盖远程分支的提交。

最终通过变基操作，顺利完成冲突的解决。