要只克隆 Git 仓库中的指定分支，可以使用 `git clone` 命令，并加上 `-b` 选项（或者 `--branch`），如下所示：

```bash
git clone -b <branch_name> --single-branch <repository_url>
```

### 说明：
- `<branch_name>`：替换为你想要克隆的分支名称。
- `<repository_url>`：替换为仓库的 URL。
- `--single-branch`：确保只克隆指定分支，而不是克隆所有分支。

例如，要只克隆 `dev` 分支，你可以运行以下命令：

```bash
git clone -b dev --single-branch https://github.com/user/repository.git
```

这样，Git 只会下载指定的分支和其相关的提交历史，节省了网络资源和存储空间。